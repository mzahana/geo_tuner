#include "geo_tuner/core/first_order_fit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "geo_tuner/core/least_squares.hpp"

namespace geo_tuner
{
namespace
{

bool pinned(double v, double lo, double hi, bool skip_lo = false)
{
  const double span = hi - lo;
  return (!skip_lo && v - lo < 0.01 * span) || (hi - v < 0.01 * span);
}

}  // namespace

Eigen::VectorXd first_order_step(const Eigen::VectorXd & t_in, double T)
{
  Eigen::VectorXd y(t_in.size());
  const double Tc = std::max(T, 1e-6);
  for (Eigen::Index i = 0; i < t_in.size(); ++i) {
    const double t = std::max(t_in[i], 0.0);
    y[i] = t_in[i] > 0.0 ? 1.0 - std::exp(-t / Tc) : 0.0;
  }
  return y;
}

FirstOrderFitResult fit_first_order(
  const Eigen::VectorXd & t, const Eigen::VectorXd & y, double step,
  double T_guess, std::pair<double, double> T_bounds)
{
  if (t.size() < 10) {
    throw std::invalid_argument("Need at least 10 samples");
  }
  if (std::abs(step) < 1e-6) {
    throw std::invalid_argument("step must be nonzero");
  }

  const Eigen::VectorXd yn = y / step;

  auto residuals = [&t, &yn](const Eigen::VectorXd & p) {
      const double T = p[0], td = p[1], amp = p[2];
      Eigen::VectorXd shifted = t.array() - td;
      return (amp * first_order_step(shifted, T) - yn).eval();
    };

  const Eigen::Vector3d lb(T_bounds.first, 0.0, 0.3);
  const Eigen::Vector3d ub(T_bounds.second, 0.4, 1.7);
  const Eigen::Vector3d p0(T_guess, 0.05, 1.0);
  const auto sol = least_squares_bounded(residuals, clip(p0, lb, ub), lb, ub);

  const double T = sol.x[0], td = sol.x[1], amp = sol.x[2];
  const bool at_bounds =
    pinned(T, lb[0], ub[0]) ||
    pinned(td, lb[1], ub[1], /*skip_lo=*/true) ||
    pinned(amp, lb[2], ub[2]);

  FirstOrderFitResult out;
  out.T = T;
  out.delay = td;
  out.amplitude = amp * step;
  out.nrmse = std::sqrt(sol.fun.squaredNorm() / static_cast<double>(sol.fun.size()));
  out.converged = sol.success;
  out.at_bounds = at_bounds;
  return out;
}

}  // namespace geo_tuner
