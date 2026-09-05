// Fit a second-order-plus-delay model to a recorded step response.
//
// Used by the in-flight tuner: after a small setpoint step on one axis,
// the position response is fit to
//
//     y(t) = A * step2(t - td; wn, zeta)
//
// where step2 is the unit step response of wn^2 / (s^2 + 2 zeta wn s + wn^2).
// The identified (wn, zeta, td) tell us where the closed-loop poles
// actually are -- absorbing thrust-map error, inner-loop lag and transport
// delay -- and correct_gains_from_identification turns that into a gain
// update.
#ifndef GEO_TUNER__CORE__STEP_FIT_HPP_
#define GEO_TUNER__CORE__STEP_FIT_HPP_

#include <Eigen/Dense>
#include <optional>
#include <utility>

namespace geo_tuner
{

struct StepFitResult
{
  double wn{};          // rad/s
  double zeta{};
  double delay{};       // s
  double amplitude{};   // fitted steady-state (should be ~ step size)
  double rmse{};        // residual RMS, same units as y
  double nrmse{};       // rmse / |step|
  double overshoot{};   // fraction of step, from the data
  bool converged{false};
  bool at_bounds{false};  // a parameter pinned at the optimizer bounds

  /// Fit quality gate used before trusting a gain update.
  ///
  /// A parameter pinned at its optimizer bound means the model could not
  /// explain the data inside the physically plausible box -- the classic
  /// wn/zeta/delay ambiguity of overdamped-looking responses. Such fits
  /// are rejected rather than acted on.
  bool ok() const
  {
    return converged && nrmse < 0.15 && zeta > 0.05 && zeta < 2.5 && !at_bounds;
  }
};

/// Unit step response of a standard second-order system (t >= 0).
Eigen::VectorXd second_order_step(const Eigen::VectorXd & t, double wn, double zeta);

/// Fit (wn, zeta, delay, amplitude) to a measured step response.
///
/// t: seconds, 0 at step command time. y: position relative to the
/// pre-step position (same sign convention as `step`).
///
/// wn_bounds: optional (lo, hi) prior on wn. In closed-loop
/// identification the applied gains predict wn up to the plant-gain
/// factor alpha, so wn is *known* to lie in
/// wn_pred * [sqrt(alpha_min), sqrt(alpha_hi)]. Constraining the fit to
/// that box resolves the (wn, zeta, delay) ambiguity of higher-order
/// responses: the optimizer explains the data with a physically possible
/// wn and lets zeta/delay absorb the inner-loop lag. wn landing on these
/// deliberate prior bounds does NOT mark the fit at_bounds.
StepFitResult fit_step_response(
  const Eigen::VectorXd & t, const Eigen::VectorXd & y, double step,
  double wn_guess = 2.0, double zeta_guess = 0.9,
  std::optional<std::pair<double, double>> wn_bounds = std::nullopt);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__STEP_FIT_HPP_
