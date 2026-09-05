#include "geo_tuner/core/step_fit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "geo_tuner/core/least_squares.hpp"

namespace geo_tuner
{
namespace
{

/// A bound is "pinned" when the solution sits within 1% of the box span
/// of it. The delay's lower bound of 0 is a legitimate value and is
/// excluded by the caller.
bool pinned(double v, double lo, double hi, bool skip_lo = false)
{
  const double span = hi - lo;
  return (!skip_lo && v - lo < 0.01 * span) || (hi - v < 0.01 * span);
}

}  // namespace

Eigen::VectorXd second_order_step(const Eigen::VectorXd & t_in, double wn, double zeta)
{
  Eigen::VectorXd y(t_in.size());
  const Eigen::VectorXd t = t_in.cwiseMax(0.0);
  if (zeta < 1.0 - 1e-9) {
    const double wd = wn * std::sqrt(1.0 - zeta * zeta);
    const double phi = std::acos(zeta);
    const double s = std::sqrt(1.0 - zeta * zeta);
    for (Eigen::Index i = 0; i < t.size(); ++i) {
      y[i] = 1.0 - std::exp(-zeta * wn * t[i]) / s * std::sin(wd * t[i] + phi);
    }
  } else if (zeta > 1.0 + 1e-9) {
    const double r = std::sqrt(zeta * zeta - 1.0);
    const double s1 = -wn * (zeta - r);
    const double s2 = -wn * (zeta + r);
    for (Eigen::Index i = 0; i < t.size(); ++i) {
      y[i] = 1.0 + (s2 * std::exp(s1 * t[i]) - s1 * std::exp(s2 * t[i])) / (s1 - s2);
    }
  } else {  // critically damped
    for (Eigen::Index i = 0; i < t.size(); ++i) {
      y[i] = 1.0 - std::exp(-wn * t[i]) * (1.0 + wn * t[i]);
    }
  }
  for (Eigen::Index i = 0; i < t_in.size(); ++i) {
    if (!(t_in[i] > 0.0)) {y[i] = 0.0;}
  }
  return y;
}

StepFitResult fit_step_response(
  const Eigen::VectorXd & t, const Eigen::VectorXd & y, double step,
  double wn_guess, double zeta_guess,
  std::optional<std::pair<double, double>> wn_bounds)
{
  if (t.size() < 10) {
    throw std::invalid_argument("Need at least 10 samples to fit a step response");
  }
  if (std::abs(step) < 1e-6) {
    throw std::invalid_argument("step must be nonzero");
  }

  const Eigen::VectorXd yn = y / step;  // normalize to unit step

  auto residuals = [&t, &yn](const Eigen::VectorXd & p) {
      const double wn = p[0], zeta = p[1], td = p[2], amp = p[3];
      Eigen::VectorXd shifted = t.array() - td;
      return (amp * second_order_step(shifted, wn, zeta) - yn).eval();
    };

  Eigen::Vector4d lb(0.1, 0.05, 0.0, 0.3);
  Eigen::Vector4d ub(30.0, 2.5, 0.6, 1.7);
  const bool wn_prior_bounds = wn_bounds.has_value();
  if (wn_prior_bounds) {
    lb[0] = wn_bounds->first;
    ub[0] = wn_bounds->second;
    wn_guess = std::clamp(wn_guess, lb[0], ub[0]);
  }

  // The (wn, zeta, delay) triple is weakly identifiable for
  // near-critically-damped responses: "high wn + overdamped + large
  // delay" mimics "wn near truth + small delay" almost exactly (the
  // response is really higher-order: inner-loop lag stacks on the
  // position loop). Resolve the ambiguity by multi-start optimization
  // and, among solutions of near-equal residual, prefer the one whose wn
  // is closest to the prior wn_guess (i.e. plant gain alpha ~ 1).
  const std::vector<Eigen::Vector3d> starts = {
    {wn_guess, zeta_guess, 0.03},
    {wn_guess, zeta_guess, 0.15},
    {0.6 * wn_guess, 0.7, 0.05},
    {1.5 * wn_guess, 1.3, 0.05},
    {wn_guess, 0.5, 0.30},
  };

  std::vector<LeastSquaresResult> sols;
  for (const auto & s : starts) {
    Eigen::Vector4d p0(s[0], s[1], s[2], 1.0);
    auto r = least_squares_bounded(residuals, clip(p0, lb, ub), lb, ub);
    if (r.success) {sols.push_back(std::move(r));}
  }
  if (sols.empty()) {
    Eigen::Vector4d p0(wn_guess, zeta_guess, 0.05, 1.0);
    sols.push_back(least_squares_bounded(residuals, clip(p0, lb, ub), lb, ub));
  }

  auto nrmse_of = [](const LeastSquaresResult & s) {
      return std::sqrt(s.fun.squaredNorm() / static_cast<double>(s.fun.size()));
    };

  double best = nrmse_of(sols.front());
  for (const auto & s : sols) {best = std::min(best, nrmse_of(s));}
  const double gate = std::max(1.25 * best, best + 0.005);

  const LeastSquaresResult * sol = nullptr;
  double best_key = 0.0;
  for (const auto & s : sols) {
    if (nrmse_of(s) > gate) {continue;}
    const double key = std::abs(std::log(std::max(s.x[0], 1e-6) / wn_guess));
    if (sol == nullptr || key < best_key) {
      sol = &s;
      best_key = key;
    }
  }

  const double wn = sol->x[0], zeta = sol->x[1], td = sol->x[2], amp = sol->x[3];

  // Bound-pinning check (delay's lower bound of 0 is a legitimate value
  // and is excluded; everything else pinned means an untrustworthy fit).
  const bool at_bounds =
    (!wn_prior_bounds && pinned(wn, lb[0], ub[0])) ||
    pinned(zeta, lb[1], ub[1]) ||
    pinned(td, lb[2], ub[2], /*skip_lo=*/true) ||
    pinned(amp, lb[3], ub[3]);

  const Eigen::VectorXd res = sol->fun;
  const double rmse =
    std::sqrt(res.squaredNorm() / static_cast<double>(res.size())) * std::abs(step);
  const double nrmse = rmse / std::abs(step);

  // Overshoot straight from the data (robust to model mismatch)
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
  const double overshoot =
    std::max(0.0, (peak - std::abs(ss)) / std::max(std::abs(ss), 1e-6));

  StepFitResult out;
  out.wn = wn;
  out.zeta = zeta;
  out.delay = td;
  out.amplitude = amp * step;
  out.rmse = rmse;
  out.nrmse = nrmse;
  out.overshoot = overshoot;
  out.converged = sol->success;
  out.at_bounds = at_bounds;
  return out;
}

}  // namespace geo_tuner
