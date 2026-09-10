#include "geo_tuner/core/loop_fit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <vector>

#include "geo_tuner/core/least_squares.hpp"
#include "geo_tuner/core/step_fit.hpp"   // second_order_step, for the tau -> 0 limit

namespace geo_tuner
{
namespace
{

using Complex = std::complex<double>;

// Numerical sanity bounds. Deliberately wide: they exist so the optimizer
// cannot wander into overflow, NOT to express what is physically
// plausible. Plausibility is a separate judgement made afterwards on an
// unconstrained estimate -- which is what makes a parameter resting on
// one of these bounds meaningful evidence that the fit failed.
constexpr double kAlphaLo = 0.05, kAlphaHi = 10.0;
constexpr double kTauLo = 0.005, kTauHi = 1.5;
constexpr double kDelayLo = 0.0, kDelayHi = 0.4;
constexpr double kAmpLo = 0.3, kAmpHi = 1.7;

/// Below this the third pole is far outside the data's bandwidth and the
/// cubic degenerates numerically; the second-order form is exact there.
constexpr double kTauNegligible = 1e-3;

bool pinned(double v, double lo, double hi, bool skip_lo = false)
{
  const double span = hi - lo;
  return (!skip_lo && v - lo < 0.01 * span) || (hi - v < 0.01 * span);
}

/// Roots of s^3 + b2 s^2 + b1 s + b0 via the companion matrix.
std::array<Complex, 3> cubic_roots(double b2, double b1, double b0)
{
  Eigen::Matrix3d C;
  C << -b2, -b1, -b0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0;
  Eigen::EigenSolver<Eigen::Matrix3d> es(C, /*computeEigenvectors=*/false);
  const auto ev = es.eigenvalues();
  return {ev(0), ev(1), ev(2)};
}

}  // namespace

Eigen::VectorXd closed_loop_step(
  const Eigen::VectorXd & t, double kx, double kv, double alpha, double tau)
{
  const double a0 = alpha * kx;
  const double a1 = alpha * kv;

  if (tau < kTauNegligible) {
    // s^2 + alpha*kv*s + alpha*kx: the design second order, exactly.
    const double wn = std::sqrt(std::max(a0, 1e-12));
    const double zeta = a1 / (2.0 * wn);
    return second_order_step(t, wn, zeta);
  }

  // Monic form of tau*s^3 + s^2 + alpha*kv*s + alpha*kx.
  auto roots = cubic_roots(1.0 / tau, a1 / tau, a0 / tau);

  // Residues blow up for repeated roots (the response is then finite but
  // the partial-fraction form is not). Nudge coincident roots apart by an
  // amount far below anything the data can resolve.
  double scale = 0.0;
  for (const auto & r : roots) {scale = std::max(scale, std::abs(r));}
  const double eps = std::max(scale, 1.0) * 1e-7;
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      if (std::abs(roots[i] - roots[j]) < eps) {
        roots[j] += Complex(eps, 0.0);
      }
    }
  }

  // y(t) = 1 + sum_k R_k exp(p_k t),  R_k = (a0/tau) / (p_k * prod_{j!=k}(p_k - p_j))
  std::array<Complex, 3> R;
  for (int k = 0; k < 3; ++k) {
    Complex den = roots[k];
    for (int j = 0; j < 3; ++j) {
      if (j != k) {den *= (roots[k] - roots[j]);}
    }
    R[k] = (std::abs(den) > 0.0) ? Complex(a0 / tau, 0.0) / den : Complex(0.0, 0.0);
  }

  Eigen::VectorXd y(t.size());
  for (Eigen::Index i = 0; i < t.size(); ++i) {
    if (!(t[i] > 0.0)) {
      y[i] = 0.0;
      continue;
    }
    Complex acc(1.0, 0.0);
    for (int k = 0; k < 3; ++k) {
      // An unstable candidate diverges; cap the exponent so the residual
      // stays finite and large, steering the optimizer away rather than
      // poisoning it with inf/nan.
      const double re = std::min(roots[k].real() * t[i], 60.0);
      const double im = roots[k].imag() * t[i];
      acc += R[k] * std::exp(Complex(re, im));
    }
    y[i] = acc.real();
  }
  return y;
}

LoopFitResult fit_closed_loop(
  const Eigen::VectorXd & t, const Eigen::VectorXd & y, double step,
  double kx, double kv, double tau_guess, bool model_output_delay)
{
  if (t.size() < 10) {
    throw std::invalid_argument("Need at least 10 samples to fit a step response");
  }
  if (std::abs(step) < 1e-6) {
    throw std::invalid_argument("step must be nonzero");
  }
  if (kx <= 0.0 || kv <= 0.0) {
    throw std::invalid_argument("kx and kv must be > 0 to identify against them");
  }

  const Eigen::VectorXd yn = y / step;   // normalize to a unit step

  auto residuals = [&t, &yn, kx, kv](const Eigen::VectorXd & p) {
      const double alpha = p[0], tau = p[1], td = p[2], amp = p[3];
      const Eigen::VectorXd shifted = t.array() - td;
      return (amp * closed_loop_step(shifted, kx, kv, alpha, tau) - yn).eval();
    };

  const Eigen::Vector4d lb(kAlphaLo, kTauLo, kDelayLo, kAmpLo);
  const Eigen::Vector4d ub(
    kAlphaHi, kTauHi, model_output_delay ? kDelayHi : kDelayLo, kAmpHi);

  // Multi-start over the two dynamic unknowns only. The old fit needed
  // five starts and a prior-nearest tie-break to escape a degenerate
  // basin; here the starts exist to confirm the minimum is unique, and
  // disagreement between near-equal minima is reported rather than
  // resolved by preference.
  const double tg = std::clamp(tau_guess, kTauLo, kTauHi);
  const std::vector<Eigen::Vector3d> starts = {
    {1.0, tg, 0.02},
    {1.0, 0.4 * tg, 0.02},
    {1.0, 2.0 * tg, 0.05},
    {0.6, tg, 0.05},
    {1.6, tg, 0.05},
  };

  std::vector<LeastSquaresResult> sols;
  for (const auto & s : starts) {
    Eigen::Vector4d p0(s[0], s[1], s[2], 1.0);
    auto r = least_squares_bounded(residuals, clip(p0, lb, ub), lb, ub);
    if (r.success) {sols.push_back(std::move(r));}
  }
  if (sols.empty()) {
    Eigen::Vector4d p0(1.0, tg, 0.02, 1.0);
    sols.push_back(least_squares_bounded(residuals, clip(p0, lb, ub), lb, ub));
  }

  auto nrmse_of = [](const LeastSquaresResult & s) {
      return std::sqrt(s.fun.squaredNorm() / static_cast<double>(s.fun.size()));
    };

  const LeastSquaresResult * best = &sols.front();
  for (const auto & s : sols) {
    if (nrmse_of(s) < nrmse_of(*best)) {best = &s;}
  }

  // Degeneracy detector. If another start reached an essentially equal
  // residual but a materially different alpha, the data does not pick one
  // -- exactly the situation the old model hid by preferring whichever
  // minimum sat nearest the prior. Refuse instead.
  const double best_n = nrmse_of(*best);
  const double tie = std::max(1.10 * best_n, best_n + 0.002);
  bool ambiguous = false;
  for (const auto & s : sols) {
    if (nrmse_of(s) <= tie &&
      std::abs(std::log(s.x[0] / best->x[0])) > std::log(1.20))
    {
      ambiguous = true;
    }
  }

  const double alpha = best->x[0], tau = best->x[1];
  const double td = best->x[2], amp = best->x[3];

  // Zero residual shift is a real answer, so the delay keeps its
  // lower-bound exemption. The lag does NOT: kTauLo sits far below any
  // lag this control path can exhibit (attitude loop + transport,
  // measured at 160-170 ms by cmd-vs-attitude cross-correlation on the
  // 2026-09-10 flight), so tau resting on the floor means the optimizer
  // spent the bound to buy fit quality -- a failed fit. That flight
  // accepted three floor fits (alpha 0.905/0.505/0.618 against 0.99-1.25
  // for clean episodes) and their spread cost the session its final rung
  // on x and z. Regression: test_core.cpp FieldReplay, data in
  // test/data/field_2026-09-10; rationale: ihunter_fixes/docs/
  // TUNER_IMPROVEMENTS_PLAN.md (T1).
  const bool at_bounds =
    pinned(alpha, lb[0], ub[0]) ||
    pinned(tau, lb[1], ub[1]) ||
    (model_output_delay && pinned(td, lb[2], ub[2], /*skip_lo=*/true)) ||
    pinned(amp, lb[3], ub[3]);

  const Eigen::VectorXd res = best->fun;
  const double rmse =
    std::sqrt(res.squaredNorm() / static_cast<double>(res.size())) * std::abs(step);

  // Overshoot straight from the data (robust to model mismatch).
  const double t_lo = t[t.size() - 1] - 0.2 * (t[t.size() - 1] - t[0]);
  double sum = 0.0;
  int n = 0;
  for (Eigen::Index i = 0; i < t.size(); ++i) {
    if (t[i] > t_lo) {
      sum += yn[i];
      ++n;
    }
  }
  const double ss = n > 0 ? sum / n : 1.0;
  const double sign = ss > 0.0 ? 1.0 : (ss < 0.0 ? -1.0 : 0.0);
  const double peak = ss != 0.0 ? (yn * sign).maxCoeff() : yn.maxCoeff();

  LoopFitResult out;
  out.alpha = alpha;
  out.tau = tau;
  out.delay = td;
  out.amplitude = amp * step;
  out.rmse = rmse;
  out.nrmse = rmse / std::abs(step);
  out.overshoot = std::max(0.0, (peak - std::abs(ss)) / std::max(std::abs(ss), 1e-6));
  out.converged = best->success;
  out.at_bounds = at_bounds;
  out.ambiguous = ambiguous;
  return out;
}

}  // namespace geo_tuner
