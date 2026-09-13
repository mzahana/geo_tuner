#include "geo_tuner/core/accel_loop_id.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace geo_tuner
{
namespace
{

struct Biquad
{
  double b0, b1, b2, a1, a2;
};

Biquad butter2_lowpass(double fc, double fs)
{
  // Bilinear transform with pre-warping.
  const double k = std::tan(M_PI * fc / fs);
  const double q = std::sqrt(2.0);
  const double norm = 1.0 / (1.0 + q * k + k * k);
  Biquad f{};
  f.b0 = k * k * norm;
  f.b1 = 2.0 * f.b0;
  f.b2 = f.b0;
  f.a1 = 2.0 * (k * k - 1.0) * norm;
  f.a2 = (1.0 - q * k + k * k) * norm;
  return f;
}

/// One direction of the biquad, state initialised at the steady state of
/// the first sample so the edge does not ring.
void biquad_pass(std::vector<double> & x, const Biquad & f)
{
  if (x.empty()) {return;}
  const double c = x.front();
  double z1 = c * (1.0 - f.b0);
  double z2 = c * (f.b2 - f.a2);
  for (double & xi : x) {
    const double in = xi;
    const double y = f.b0 * in + z1;
    z1 = f.b1 * in - f.a1 * y + z2;
    z2 = f.b2 * in - f.a2 * y;
    xi = y;
  }
}

/// Instrument shifts [s]. The nominal command z is known ahead of time (the
/// setpoint schedule), and the gust is independent of it, so any shift of z is
/// a valid instrument. Several shifts span the delayed and lagged responses
/// the grid considers, which over-identifies the model and lets the lag be
/// chosen on the part of the data the setpoint explains (see select()).
const double kInstrumentShifts[] = {-0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4};
constexpr int kM = sizeof(kInstrumentShifts) / sizeof(kInstrumentShifts[0]);
using Vec = Eigen::Matrix<double, kM, 1>;
using Mat = Eigen::Matrix<double, kM, kM>;

/// Per-block moments shared by every combination.
struct Shared
{
  Mat zz{Mat::Zero()};
  Vec zy{Vec::Zero()};
  void add(const Shared & o) {zz += o.zz; zy += o.zy;}
  void sub(const Shared & o) {zz -= o.zz; zy -= o.zy;}
};

/// Per-combination, per-block sufficient statistics.
struct Sums
{
  Vec zx{Vec::Zero()};          // instrument moments of the candidate regressor
  double yy{0}, xy{0}, xx{0};   // residual moments on excited samples (for r2)
  void add(const Sums & o) {zx += o.zx; yy += o.yy; xy += o.xy; xx += o.xx;}
  void sub(const Sums & o) {zx -= o.zx; yy -= o.yy; xy -= o.xy; xx -= o.xx;}
};

struct Pick
{
  int combo{-1};
  double alpha{0.0};
  double sse{std::numeric_limits<double>::infinity()};   // plain residual
  double r2_iv{0.0};   // share of the setpoint-explained acceleration the model explains
  double lag{0.0};   // continuous total lag [s]
};

/// Best (delay, lag) combination by the two-stage least-squares criterion,
/// and a CONTINUOUS total lag.
///
/// Why 2SLS and not the plain residual: under feedback the command reacts to
/// the gust, so the plain residual ||y - alpha x|| is smallest at a lag that
/// lines the command's reaction up with the gust, not at the plant's lag --
/// in wind it read the lag short, the unsafe direction (quad_sim 2026-09-13:
/// 180 ms against 215). Projecting the residual onto the instruments keeps
/// only what the setpoint explains, which the gust cannot bias:
///   alpha = x'Py / x'Px,  J = y'Py - (x'Py)^2 / x'Px,  P = Z (Z'Z)^-1 Z'.
///
/// The criterion is profiled over total lag (delay + tau, the quantity that
/// costs phase) and the minimum refined by a parabola through its neighbours.
/// The jackknife needs a smooth statistic -- an argmin over a grid jumps
/// between grid points from one replicate to the next, which made lag
/// intervals alternately collapse to one grid step and balloon.
Pick select(
  const std::vector<Sums> & totals, const Shared & shared, size_t n_lags, double step)
{
  Pick best;
  const double ridge = 1e-9 * std::max(shared.zz.trace(), 1e-12);
  const Eigen::LDLT<Mat> ldlt(shared.zz + ridge * Mat::Identity());
  const Vec sc = ldlt.solve(shared.zy);
  const double csc = shared.zy.dot(sc);
  const size_t n_delays = totals.size() / n_lags;
  // Delays and lags share the grid step, so combo (di, li) has total lag
  // di + li steps.
  const size_t n_bins = (n_delays - 1) + n_lags;
  std::vector<double> profile(n_bins, std::numeric_limits<double>::infinity());
  double best_j = std::numeric_limits<double>::infinity();
  for (size_t c = 0; c < totals.size(); ++c) {
    const Sums & s = totals[c];
    const double bsc = s.zx.dot(sc);
    const double bsb = s.zx.dot(ldlt.solve(s.zx));
    if (!(bsb > 1e-12)) {continue;}
    const double a = bsc / bsb;
    if (!(a > 0.0) || !std::isfinite(a)) {continue;}
    const double j = csc - bsc * a;
    const size_t bin = c / n_lags + c % n_lags;
    profile[bin] = std::min(profile[bin], j);
    if (j < best_j) {
      best_j = j;
      best.combo = static_cast<int>(c);
      best.alpha = a;
      best.sse = s.yy - 2.0 * a * s.xy + a * a * s.xx;
      best.r2_iv = csc > 0.0 ? 1.0 - j / csc : 0.0;
    }
  }
  if (best.combo < 0) {return best;}
  const size_t k = static_cast<size_t>(best.combo) / n_lags +
    static_cast<size_t>(best.combo) % n_lags;
  double frac = 0.0;
  if (k > 0 && k + 1 < n_bins && std::isfinite(profile[k - 1]) && std::isfinite(profile[k + 1])) {
    const double ym = profile[k - 1], y0 = profile[k], yp = profile[k + 1];
    const double den = ym - 2.0 * y0 + yp;
    if (den > 0.0) {frac = std::clamp(0.5 * (ym - yp) / den, -0.5, 0.5);}
  }
  best.lag = (static_cast<double>(k) + frac) * step;
  return best;
}

}  // namespace

std::vector<double> filtfilt_lowpass(const std::vector<double> & x, double fc, double fs)
{
  const size_t n = x.size();
  if (n < 4 || fc <= 0.0 || fc >= 0.5 * fs) {return x;}
  const Biquad f = butter2_lowpass(fc, fs);
  // Odd extension at both ends, then forward-backward.
  const size_t pad = std::min(n - 1, static_cast<size_t>(std::ceil(2.0 * fs / fc)));
  std::vector<double> e;
  e.reserve(n + 2 * pad);
  for (size_t i = pad; i >= 1; --i) {e.push_back(2.0 * x.front() - x[i]);}
  e.insert(e.end(), x.begin(), x.end());
  for (size_t i = 1; i <= pad; ++i) {e.push_back(2.0 * x.back() - x[n - 1 - i]);}
  biquad_pass(e, f);
  std::reverse(e.begin(), e.end());
  biquad_pass(e, f);
  std::reverse(e.begin(), e.end());
  return std::vector<double>(e.begin() + static_cast<std::ptrdiff_t>(pad),
           e.begin() + static_cast<std::ptrdiff_t>(pad + n));
}

std::vector<double> nominal_command(
  const std::vector<double> & r, const std::vector<double> & kx,
  const std::vector<double> & kv, double fs)
{
  std::vector<double> out(r.size(), 0.0);
  if (r.empty()) {return out;}
  constexpr int kSub = 5;
  const double h = 1.0 / (fs * kSub);
  double p = r.front(), pd = 0.0;
  for (size_t i = 0; i < r.size(); ++i) {
    double acc = 0.0;
    for (int k = 0; k < kSub; ++k) {
      acc = kx[i] * (r[i] - p) - kv[i] * pd;
      pd += h * acc;        // semi-implicit Euler: stable for these gains
      p += h * pd;
    }
    out[i] = kx[i] * (r[i] - p) - kv[i] * pd;
  }
  return out;
}

double t_quantile(double confidence, int dof)
{
  // Two-sided quantile. Table for small dof (where the approximation below
  // is poor), Cornish-Fisher expansion of the normal quantile otherwise.
  static const double t80[] = {3.078, 1.886, 1.638, 1.533, 1.476, 1.440, 1.415, 1.397, 1.383};
  static const double t90[] = {6.314, 2.920, 2.353, 2.132, 2.015, 1.943, 1.895, 1.860, 1.833};
  static const double t95[] = {12.706, 4.303, 3.182, 2.776, 2.571, 2.447, 2.365, 2.306, 2.262};
  const double *tab = t90;
  double zq = 1.6449;
  if (confidence < 0.85) {tab = t80; zq = 1.2816;} else if (confidence > 0.925) {
    tab = t95; zq = 1.9600;
  }
  dof = std::max(dof, 1);
  if (dof <= 9) {return tab[dof - 1];}
  const double d = static_cast<double>(dof);
  const double z3 = zq * zq * zq, z5 = z3 * zq * zq;
  return zq + (z3 + zq) / (4.0 * d) + (5.0 * z5 + 16.0 * z3 + 3.0 * zq) / (96.0 * d * d);
}

AccelLoopResult identify_accel_loop(
  const std::vector<AxisSegment> & segments, const AccelLoopConfig & cfg)
{
  AccelLoopResult out;
  const double fs = cfg.fs;
  const int max_dn = static_cast<int>(std::lround(cfg.delay_max * fs));
  const int step_n = std::max(1, static_cast<int>(std::lround(cfg.grid_step * fs)));
  std::vector<int> delays;
  for (int d = 0; d <= max_dn; d += step_n) {delays.push_back(d);}
  std::vector<double> lags;
  for (double l = 0.0; l <= cfg.lag_max + 1e-9; l += cfg.grid_step) {lags.push_back(l);}
  const size_t n_combo = delays.size() * lags.size();

  // Per-segment prepared signals: instrument z, measured accel y, filtered
  // command uf, and the index range whose samples every combination can use.
  struct Prep
  {
    std::vector<double> z, y, uf;
    size_t i0{0}, i1{0};
  };
  std::vector<Prep> prep;
  const size_t edge = static_cast<size_t>(std::lround(0.5 * fs));
  int shifts[kM];
  size_t shift_back = 0, shift_fwd = 0;
  for (int m = 0; m < kM; ++m) {
    shifts[m] = static_cast<int>(std::lround(kInstrumentShifts[m] * fs));
    if (shifts[m] > 0) {shift_back = std::max(shift_back, static_cast<size_t>(shifts[m]));}
    if (shifts[m] < 0) {shift_fwd = std::max(shift_fwd, static_cast<size_t>(-shifts[m]));}
  }
  for (const auto & s : segments) {
    const size_t n = s.r.size();
    if (n != s.v.size() || n != s.u.size() || n != s.kx.size() || n != s.kv.size()) {
      out.reason = "segment arrays differ in length";
      return out;
    }
    const size_t skip = std::max({static_cast<size_t>(max_dn), edge, shift_back});
    const size_t tail = std::max(edge, shift_fwd);
    if (n < skip + tail + static_cast<size_t>(fs)) {continue;}   // too short to use
    Prep p;
    p.z = filtfilt_lowpass(nominal_command(s.r, s.kx, s.kv, fs), cfg.lpf_hz, fs);
    const auto vf = filtfilt_lowpass(s.v, cfg.lpf_hz, fs);
    p.y.resize(n);
    for (size_t i = 0; i < n; ++i) {
      const size_t a = i == 0 ? 0 : i - 1, b = std::min(n - 1, i + 1);
      p.y[i] = (vf[b] - vf[a]) * fs / static_cast<double>(b - a);
    }
    p.uf = filtfilt_lowpass(s.u, cfg.lpf_hz, fs);
    p.i0 = skip;
    p.i1 = n - tail;
    prep.push_back(std::move(p));
  }
  if (prep.empty()) {
    out.reason = "no recording long enough to identify from";
    return out;
  }

  // Centre y and z per segment (a constant trim/bias carries no dynamics),
  // and find the excited samples from the instrument alone -- the mask must
  // not depend on the candidate model.
  double zmax = 0.0;
  for (auto & p : prep) {
    for (auto * sig : {&p.y, &p.z}) {
      double m = 0.0;
      for (size_t i = p.i0; i < p.i1; ++i) {m += (*sig)[i];}
      m /= static_cast<double>(p.i1 - p.i0);
      for (double & v : *sig) {v -= m;}
    }
    for (size_t i = p.i0; i < p.i1; ++i) {zmax = std::max(zmax, std::abs(p.z[i]));}
  }
  if (!(zmax > 1e-6)) {
    out.reason = "the setpoint never moved on this axis: nothing excites the loop";
    return out;
  }
  const double z_thr = cfg.excite_frac * zmax;

  // Contiguous blocks holding equal numbers of excited samples.
  size_t n_excited = 0, n_valid = 0;
  for (const auto & p : prep) {
    for (size_t i = p.i0; i < p.i1; ++i) {
      ++n_valid;
      if (std::abs(p.z[i]) > z_thr) {++n_excited;}
    }
  }
  out.n_samples = static_cast<int>(n_valid);
  out.excited_s = static_cast<double>(n_excited) / fs;
  if (out.excited_s < cfg.min_excited_s) {
    out.reason = "only " + std::to_string(out.excited_s).substr(0, 4) +
      " s of excited data (need " + std::to_string(cfg.min_excited_s).substr(0, 4) +
      " s): fly more steps on this axis";
    return out;
  }
  const int n_blocks = std::max(2, cfg.n_blocks);
  std::vector<std::vector<int>> block_of(prep.size());
  {
    size_t seen = 0;
    int current = 0;
    for (size_t sidx = 0; sidx < prep.size(); ++sidx) {
      const auto & p = prep[sidx];
      block_of[sidx].assign(p.z.size(), 0);
      for (size_t i = p.i0; i < p.i1; ++i) {
        if (std::abs(p.z[i]) > z_thr) {
          current = std::min(
            n_blocks - 1,
            static_cast<int>((seen * static_cast<size_t>(n_blocks)) / n_excited));
          ++seen;
        }
        block_of[sidx][i] = current;
      }
    }
  }

  // Sufficient statistics for every (delay, lag) combination and block.
  // Instrument m at sample i is z[i - shifts[m]] (positive shift = past).
  std::vector<std::vector<Sums>> per_block(n_blocks, std::vector<Sums>(n_combo));
  std::vector<Shared> shared_block(n_blocks);
  std::vector<double> x;
  for (size_t sidx = 0; sidx < prep.size(); ++sidx) {
    const auto & p = prep[sidx];
    const size_t n = p.uf.size();
    std::vector<Vec> zi(n, Vec::Zero());
    for (size_t i = p.i0; i < p.i1; ++i) {
      for (int m = 0; m < kM; ++m) {
        zi[i][m] = p.z[static_cast<size_t>(static_cast<std::ptrdiff_t>(i) - shifts[m])];
      }
      Shared & sh = shared_block[block_of[sidx][i]];
      sh.zz.noalias() += zi[i] * zi[i].transpose();
      sh.zy += zi[i] * p.y[i];
    }
    x.resize(n);
    for (size_t di = 0; di < delays.size(); ++di) {
      const int dn = delays[di];
      for (size_t li = 0; li < lags.size(); ++li) {
        const double tau = lags[li];
        // x[i] = a x[i-1] + (1-a) in[i] delays by a/(1-a) samples at low
        // frequency; choosing a = fs tau / (1 + fs tau) makes that exactly
        // tau. The exp(-1/(fs tau)) pole delays by less (41 ms for a 50 ms
        // lag), which read every lag ~10 ms long.
        const double a = fs * tau / (1.0 + fs * tau);
        double state = p.uf[0];
        for (size_t i = 0; i < n; ++i) {
          const double in = p.uf[i >= static_cast<size_t>(dn) ? i - dn : 0];
          state = a * state + (1.0 - a) * in;
          x[i] = state;
        }
        double mx = 0.0;
        for (size_t i = p.i0; i < p.i1; ++i) {mx += x[i];}
        mx /= static_cast<double>(p.i1 - p.i0);
        const size_t combo = di * lags.size() + li;
        for (size_t i = p.i0; i < p.i1; ++i) {
          const double xc = x[i] - mx;
          Sums & s = per_block[block_of[sidx][i]][combo];
          s.zx += zi[i] * xc;
          if (std::abs(p.z[i]) > z_thr) {
            s.yy += p.y[i] * p.y[i];
            s.xy += xc * p.y[i];
            s.xx += xc * xc;
          }
        }
      }
    }
  }
  std::vector<Sums> total(n_combo);
  Shared shared_total;
  for (int b = 0; b < n_blocks; ++b) {
    for (size_t c = 0; c < n_combo; ++c) {total[c].add(per_block[b][c]);}
    shared_total.add(shared_block[b]);
  }

  const Pick full = select(total, shared_total, lags.size(), cfg.grid_step);
  if (full.combo < 0) {
    out.reason = "no candidate model gave a positive plant gain";
    return out;
  }
  const auto delay_of = [&](int combo) {
      return static_cast<double>(delays[static_cast<size_t>(combo) / lags.size()]) / fs;
    };
  const auto tau_of = [&](int combo) {
      return lags[static_cast<size_t>(combo) % lags.size()];
    };
  out.alpha = full.alpha;
  out.delay = delay_of(full.combo);
  out.tau = tau_of(full.combo);
  out.lag = full.lag;
  // Fit quality on what the setpoint explains, not on the raw acceleration:
  // the gust is part of the raw signal, so a raw R^2 falls with wind power
  // however right the model is (z in quad_sim gusts on soft gains: 0.47-0.56,
  // refused every round). The projection removes the gust, as it does for
  // the estimate itself; what is left unexplained is model error.
  const double yy = total[static_cast<size_t>(full.combo)].yy;
  out.r2 = full.r2_iv;
  out.r2_raw = yy > 0.0 ? 1.0 - full.sse / yy : 0.0;

  // Delete-one-block jackknife.
  std::vector<double> la, ll;
  for (int b = 0; b < n_blocks; ++b) {
    std::vector<Sums> t = total;
    for (size_t c = 0; c < n_combo; ++c) {t[c].sub(per_block[b][c]);}
    Shared sh = shared_total;
    sh.sub(shared_block[b]);
    const Pick pk = select(t, sh, lags.size(), cfg.grid_step);
    if (pk.combo < 0) {continue;}
    la.push_back(std::log(pk.alpha));
    ll.push_back(pk.lag);
  }
  const auto jack_se = [](const std::vector<double> & v) {
      const double n = static_cast<double>(v.size());
      double m = 0.0;
      for (double e : v) {m += e;}
      m /= n;
      double ss = 0.0;
      for (double e : v) {ss += (e - m) * (e - m);}
      return std::sqrt((n - 1.0) / n * ss);
    };
  if (la.size() < 2) {
    out.reason = "jackknife failed: too few usable blocks";
    return out;
  }
  const double tq = t_quantile(cfg.confidence, static_cast<int>(la.size()) - 1);
  const double se_la = jack_se(la), se_l = jack_se(ll);
  out.alpha_lo = out.alpha * std::exp(-tq * se_la);
  out.alpha_hi = out.alpha * std::exp(tq * se_la);
  // Floor on the half-width: a quarter grid step is below what the grid and
  // the 50 Hz data can resolve, whatever the replicates happen to agree on.
  const double hw_l = std::max(tq * se_l, 0.25 * cfg.grid_step);
  out.lag_lo = std::max(0.0, out.lag - hw_l);
  out.lag_hi = out.lag + hw_l;

  if (out.r2 < cfg.min_r2) {
    out.reason = "model explains only " + std::to_string(out.r2).substr(0, 4) +
      " of the excited acceleration (need " + std::to_string(cfg.min_r2).substr(0, 4) + ")";
    return out;
  }
  out.ok = true;
  return out;
}

}  // namespace geo_tuner
