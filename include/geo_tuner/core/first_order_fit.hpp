// Fit a first-order-plus-delay model to a yaw step response.
//
// The yaw closed loop under the geometric controller is (small angles)
//
//     psi_dot = (1/T) * (psi_ref - psi),   T ~ yawctrl_tau / 2  (+ rate lag)
//
// so a reference step gives  psi(t) = A * (1 - exp(-(t - td)/T)).
// Identifying T tells us the *effective* yaw time constant, and
//
//     yawctrl_tau_new = yawctrl_tau_applied * T_target / T_measured
//
// places it at the target (the applied tau and measured T are
// proportional through the same unknown efficiency factor, which cancels).
#ifndef GEO_TUNER__CORE__FIRST_ORDER_FIT_HPP_
#define GEO_TUNER__CORE__FIRST_ORDER_FIT_HPP_

#include <Eigen/Dense>
#include <utility>

namespace geo_tuner
{

struct FirstOrderFitResult
{
  double T{};          // time constant [s]
  double delay{};      // s
  double amplitude{};  // fitted steady state (~ step)
  double nrmse{};
  bool converged{false};
  bool at_bounds{false};

  bool ok() const {return converged && nrmse < 0.15 && !at_bounds;}
};

Eigen::VectorXd first_order_step(const Eigen::VectorXd & t, double T);

/// Fit (T, delay, amplitude); y is the response relative to the pre-step
/// value, same sign convention as `step`.
FirstOrderFitResult fit_first_order(
  const Eigen::VectorXd & t, const Eigen::VectorXd & y, double step,
  double T_guess = 0.3, std::pair<double, double> T_bounds = {0.02, 2.0});

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__FIRST_ORDER_FIT_HPP_
