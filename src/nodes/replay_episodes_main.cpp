// CLI: replay a flown tuning session's episode dumps through the fitter.
//
// Usage:
//     geo-tuner-replay EPISODE_DIR REPORT_YAML
//         [--tau-guess 0.3] [--fixed-tau median|SECONDS]
//
// EPISODE_DIR holds the conductor's per-episode CSVs (episode_dump_dir),
// REPORT_YAML the session report that flew them -- it supplies the gains
// each episode was flown with (kx_applied/kv_applied), the wn ladder and
// zeta_target, so the replay judges every episode and bucket with exactly
// the arithmetic the conductor used in the air.
//
// This exists so a fitter change can be proven against real flight data
// before it flies: run it on a session at the old code, change the fitter,
// run it again, and diff the verdicts. --fixed-tau refits the position
// episodes with the in-loop lag frozen (per-axis median of the accepted
// free fits, or a given value), previewing the "lag is an airframe
// property, identify it once" strategy without touching the conductor.
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include "geo_tuner/core/aggregate.hpp"
#include "geo_tuner/core/first_order_fit.hpp"
#include "geo_tuner/core/gain_design.hpp"
#include "geo_tuner/core/least_squares.hpp"
#include "geo_tuner/core/loop_fit.hpp"

namespace
{

// The conductor's episode gates, mirrored (tuning_conductor.hpp / .cpp).
constexpr double kAlphaMin = 0.4, kAlphaMax = 2.5;
constexpr double kYawFitNrmse = 0.25;
constexpr double kConsistency = 1.35;
constexpr double kMaxChange = 1.6;         // max_gain_change_factor
constexpr double kStabilityMargin = 4.0;
constexpr double kYawTauMin = 0.15, kYawTauMax = 1.2;

struct Episode
{
  std::string file;
  std::string axis;
  int rung{};
  int rep{};
  double step{};
  Eigen::VectorXd t, y;
  // From the report: what the controller was running when this flew.
  double kx{}, kv{};        // position axes
  double yaw_tau{};         // yaw
};

[[noreturn]] void fail(const std::string & msg)
{
  std::cerr << "geo-tuner-replay: error: " << msg << "\n";
  std::exit(2);
}

// "# axis=z step=1 rung=1 rep=1" -> key/value map.
std::map<std::string, std::string> parse_header(const std::string & line)
{
  std::map<std::string, std::string> kv;
  std::istringstream ss(line);
  std::string tok;
  while (ss >> tok) {
    const auto eq = tok.find('=');
    if (eq != std::string::npos) {
      kv[tok.substr(0, eq)] = tok.substr(eq + 1);
    }
  }
  return kv;
}

Episode load_csv(const std::filesystem::path & path)
{
  std::ifstream f(path);
  if (!f) {fail("cannot read " + path.string());}
  Episode ep;
  ep.file = path.filename().string();
  std::string line;
  std::vector<double> ts, ys;
  while (std::getline(f, line)) {
    if (line.empty()) {continue;}
    if (line[0] == '#') {
      const auto kv = parse_header(line);
      if (kv.count("axis")) {ep.axis = kv.at("axis");}
      if (kv.count("step")) {ep.step = std::stod(kv.at("step"));}
      if (kv.count("rung")) {ep.rung = std::stoi(kv.at("rung"));}
      if (kv.count("rep")) {ep.rep = std::stoi(kv.at("rep"));}
      continue;
    }
    if (!std::isdigit(static_cast<unsigned char>(line[0])) && line[0] != '-') {
      continue;   // the "t,y" column header
    }
    const auto comma = line.find(',');
    if (comma == std::string::npos) {continue;}
    ts.push_back(std::stod(line.substr(0, comma)));
    ys.push_back(std::stod(line.substr(comma + 1)));
  }
  ep.t = Eigen::Map<Eigen::VectorXd>(ts.data(), static_cast<Eigen::Index>(ts.size()));
  ep.y = Eigen::Map<Eigen::VectorXd>(ys.data(), static_cast<Eigen::Index>(ys.size()));
  if (ep.axis.empty() || ep.t.size() == 0) {
    fail(path.string() + ": missing '# axis=...' header or samples");
  }
  return ep;
}

// Applied gains per (axis, rung, rep) out of the session report.
struct ReportData
{
  std::map<std::tuple<std::string, int, int>, std::pair<double, double>> gains;
  std::map<std::tuple<std::string, int, int>, double> yaw_tau;
  std::vector<double> wn_ladder;
  double zeta_target{0.95};
};

ReportData load_report(const std::string & path)
{
  ReportData r;
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    fail(std::string("cannot parse report: ") + e.what());
  }
  for (const auto & wn : doc["wn_ladder"]) {r.wn_ladder.push_back(wn.as<double>());}
  if (doc["zeta_target"]) {r.zeta_target = doc["zeta_target"].as<double>();}
  for (const auto & rec : doc["episodes"]) {
    if (!rec["rep"]) {continue;}                 // bucket summaries have no rep
    const auto key = std::make_tuple(
      rec["axis"].as<std::string>(), rec["rung"].as<int>(), rec["rep"].as<int>());
    if (rec["kx_applied"]) {
      r.gains[key] = {rec["kx_applied"].as<double>(), rec["kv_applied"].as<double>()};
    } else if (rec["yaw_tau_applied"]) {
      r.yaw_tau[key] = rec["yaw_tau_applied"].as<double>();
    }
  }
  return r;
}

std::string fmt(double v, int prec)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
  return buf;
}

// The conductor's per-episode verdict on a position fit.
std::string verdict(const geo_tuner::LoopFitResult & fit)
{
  if (!fit.converged) {return "REJECT no-converge";}
  if (fit.ambiguous) {return "REJECT ambiguous";}
  if (fit.at_bounds) {return "REJECT at-bounds";}
  if (fit.nrmse >= 0.15) {return "REJECT nrmse";}
  if (fit.alpha < kAlphaMin || fit.alpha > kAlphaMax) {return "REJECT alpha";}
  return "accepted";
}

// Refit one position episode with the in-loop lag frozen: LM over
// (alpha, delay, amplitude) only, same residual as fit_closed_loop.
geo_tuner::LoopFitResult refit_fixed_tau(const Episode & ep, double tau)
{
  const Eigen::VectorXd yn = ep.y / ep.step;
  auto residuals = [&](const Eigen::VectorXd & p) {
      const Eigen::VectorXd shifted = ep.t.array() - p[1];
      return (p[2] * geo_tuner::closed_loop_step(shifted, ep.kx, ep.kv, p[0], tau) -
             yn).eval();
    };
  const Eigen::Vector3d lb(0.05, 0.0, 0.3), ub(10.0, 0.4, 1.7);
  geo_tuner::LeastSquaresResult best;
  for (double a0 : {0.6, 1.0, 1.6}) {
    auto r = geo_tuner::least_squares_bounded(
      residuals, geo_tuner::clip(Eigen::Vector3d(a0, 0.02, 1.0), lb, ub), lb, ub);
    if (!best.success || (r.success && r.cost < best.cost)) {best = std::move(r);}
  }
  geo_tuner::LoopFitResult out;
  out.alpha = best.x[0];
  out.tau = tau;
  out.delay = best.x[1];
  out.amplitude = best.x[2] * ep.step;
  const double rmse =
    std::sqrt(best.fun.squaredNorm() / static_cast<double>(best.fun.size()));
  out.rmse = rmse * std::abs(ep.step);
  out.nrmse = rmse;
  out.converged = best.success;
  out.at_bounds = (best.x[0] - 0.05 < 0.01 * (10.0 - 0.05)) ||
    (10.0 - best.x[0] < 0.01 * (10.0 - 0.05));
  return out;
}

struct BucketRow
{
  std::string axis;
  int rung{};
  std::vector<double> alphas, lags;
  double kx{}, kv{};
};

void finalize_bucket(const BucketRow & b, double wn_target, double zeta_target)
{
  const auto est = geo_tuner::robust_ratio_estimate(b.alphas, 2, kConsistency);
  std::string line = "  " + b.axis + " rung " + std::to_string(b.rung) +
    " (wn_target " + fmt(wn_target, 2) + "): n_used=" + std::to_string(est.n_used);
  if (std::isfinite(est.spread)) {line += " spread=" + fmt(est.spread, 2) + "x";}
  if (!est.ok) {
    std::cout << line << " -> KEEPING GAINS (" << est.reason << ")\n";
    return;
  }
  const double tau_med = geo_tuner::median(b.lags);
  const double wn_cap = tau_med > 0.0 ?
    2.0 * zeta_target / (kStabilityMargin * tau_med) :
    std::numeric_limits<double>::infinity();
  if (wn_target > wn_cap) {
    std::cout << line << " -> LADDER STOPPED (wn_target " << fmt(wn_target, 2) <<
      " > stability cap " << fmt(wn_cap, 2) << " from lag " <<
      fmt(tau_med * 1e3, 0) << " ms)\n";
    return;
  }
  const auto corr = geo_tuner::correct_gains_from_identification(
    b.kx, std::sqrt(est.value * b.kx), wn_target, zeta_target);
  const double kx_new = std::clamp(corr.kx_new, b.kx / kMaxChange, b.kx * kMaxChange);
  const double kv_new = std::clamp(corr.kv_new, b.kv / kMaxChange, b.kv * kMaxChange);
  std::cout << line << " alpha=" << fmt(est.value, 3) <<
    " -> kx " << fmt(b.kx, 3) << "->" << fmt(kx_new, 3) <<
    ", kv " << fmt(b.kv, 3) << "->" << fmt(kv_new, 3) << "\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string dir, report_path, fixed_tau_arg;
  double tau_guess = 0.3;   // the flight config's attctrl_tau
  std::vector<std::string> pos_args;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--tau-guess" && i + 1 < argc) {tau_guess = std::stod(argv[++i]);} else
    if (a == "--fixed-tau" && i + 1 < argc) {fixed_tau_arg = argv[++i];} else
    if (a == "-h" || a == "--help") {
      std::cout << "usage: geo-tuner-replay EPISODE_DIR REPORT_YAML"
        " [--tau-guess 0.3] [--fixed-tau median|SECONDS]\n";
      return 0;
    } else {pos_args.push_back(a);}
  }
  if (pos_args.size() != 2) {fail("expected EPISODE_DIR and REPORT_YAML (see --help)");}
  dir = pos_args[0];
  report_path = pos_args[1];

  const ReportData report = load_report(report_path);

  std::vector<std::filesystem::path> files;
  for (const auto & e : std::filesystem::directory_iterator(dir)) {
    if (e.path().extension() == ".csv") {files.push_back(e.path());}
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) {fail("no .csv episodes in " + dir);}

  std::map<std::pair<std::string, int>, BucketRow> buckets;
  std::vector<Episode> position_eps;
  std::map<std::pair<int, int>, std::vector<double>> yaw_T;   // (rung) -> T list

  std::printf("%-24s %7s %7s %8s %7s %5s  %s\n",
    "episode", "alpha", "tau_ms", "nrmse", "os_pct", "n", "verdict");
  for (const auto & f : files) {
    Episode ep = load_csv(f);
    const auto key = std::make_tuple(ep.axis, ep.rung, ep.rep);
    if (ep.axis == "yaw") {
      const auto it = report.yaw_tau.find(key);
      if (it == report.yaw_tau.end()) {
        std::printf("%-24s  not in report; skipped\n", ep.file.c_str());
        continue;
      }
      ep.yaw_tau = it->second;
      const auto fit = geo_tuner::fit_first_order(
        ep.t, ep.y, ep.step, std::max(ep.yaw_tau / 2.0, 0.05));
      const bool ok = fit.converged && !fit.at_bounds && fit.nrmse < kYawFitNrmse;
      std::printf("%-24s %7s %7s %8s %7s %5d  %s\n", ep.file.c_str(),
        ("T=" + fmt(fit.T, 2)).c_str(), fmt(fit.delay * 1e3, 0).c_str(),
        fmt(fit.nrmse, 3).c_str(), "-", static_cast<int>(ep.t.size()),
        ok ? "accepted" : "REJECT yaw-gate");
      if (ok) {yaw_T[{ep.rung, 0}].push_back(fit.T);}
      continue;
    }
    const auto it = report.gains.find(key);
    if (it == report.gains.end()) {
      std::printf("%-24s  not in report; skipped\n", ep.file.c_str());
      continue;
    }
    ep.kx = it->second.first;
    ep.kv = it->second.second;
    const auto fit = geo_tuner::fit_closed_loop(
      ep.t, ep.y, ep.step, ep.kx, ep.kv, tau_guess);
    const std::string v = verdict(fit);
    std::printf("%-24s %7s %7s %8s %7s %5d  %s\n", ep.file.c_str(),
      fmt(fit.alpha, 3).c_str(), fmt(fit.tau * 1e3, 0).c_str(),
      fmt(fit.nrmse, 3).c_str(), fmt(fit.overshoot * 100, 0).c_str(),
      static_cast<int>(ep.t.size()), v.c_str());
    auto & b = buckets[{ep.axis, ep.rung}];
    b.axis = ep.axis;
    b.rung = ep.rung;
    b.kx = ep.kx;
    b.kv = ep.kv;
    if (v == "accepted") {
      b.alphas.push_back(fit.alpha);
      b.lags.push_back(fit.tau);
    }
    position_eps.push_back(std::move(ep));
  }

  std::cout << "\nbuckets (conductor arithmetic: consistency " << kConsistency <<
    "x, rate clamp " << kMaxChange << "x, stability margin " << kStabilityMargin <<
    "x):\n";
  for (const auto & [key, b] : buckets) {
    const double wn_target = (b.rung < static_cast<int>(report.wn_ladder.size())) ?
      report.wn_ladder[b.rung] : 0.0;
    finalize_bucket(b, wn_target, report.zeta_target);
  }
  for (const auto & [key, Ts] : yaw_T) {
    const auto est = geo_tuner::robust_ratio_estimate(Ts, 2, kConsistency);
    if (est.ok) {
      std::cout << "  yaw rung " << key.first << ": median T=" << fmt(est.value, 3) <<
        " s (n=" << est.n_used << ", spread " << fmt(est.spread, 2) << "x)\n";
    } else {
      std::cout << "  yaw rung " << key.first << ": " << est.reason << "\n";
    }
  }

  if (!fixed_tau_arg.empty()) {
    // Per-axis frozen lag: the T2 preview. Median over this session's
    // accepted free fits unless a numeric value was given.
    std::map<std::string, double> axis_tau;
    if (fixed_tau_arg != "median") {
      const double v = std::stod(fixed_tau_arg);
      for (const auto & ep : position_eps) {axis_tau[ep.axis] = v;}
    } else {
      std::map<std::string, std::vector<double>> lags;
      for (const auto & [key, b] : buckets) {
        for (double l : b.lags) {lags[b.axis].push_back(l);}
      }
      for (auto & [ax, ls] : lags) {axis_tau[ax] = geo_tuner::median(ls);}
    }
    std::cout << "\n--fixed-tau refit (lag frozen per axis:";
    for (const auto & [ax, tv] : axis_tau) {
      std::cout << " " << ax << "=" << fmt(tv * 1e3, 0) << "ms";
    }
    std::cout << "):\n";
    std::map<std::pair<std::string, int>, BucketRow> fbuckets;
    for (const auto & ep : position_eps) {
      const auto fit = refit_fixed_tau(ep, axis_tau.at(ep.axis));
      const bool ok = fit.converged && !fit.at_bounds && fit.nrmse < 0.15 &&
        fit.alpha >= kAlphaMin && fit.alpha <= kAlphaMax;
      std::printf("%-24s %7s %7s %8s %7s %5s  %s\n", ep.file.c_str(),
        fmt(fit.alpha, 3).c_str(), fmt(fit.tau * 1e3, 0).c_str(),
        fmt(fit.nrmse, 3).c_str(), "-", "-",
        ok ? "accepted" : "REJECT");
      auto & b = fbuckets[{ep.axis, ep.rung}];
      b.axis = ep.axis;
      b.rung = ep.rung;
      b.kx = ep.kx;
      b.kv = ep.kv;
      if (ok) {
        b.alphas.push_back(fit.alpha);
        b.lags.push_back(fit.tau);
      }
    }
    std::cout << "fixed-tau buckets:\n";
    for (const auto & [key, b] : fbuckets) {
      const double wn_target = (b.rung < static_cast<int>(report.wn_ladder.size())) ?
        report.wn_ladder[b.rung] : 0.0;
      finalize_bucket(b, wn_target, report.zeta_target);
    }
  }
  return 0;
}
