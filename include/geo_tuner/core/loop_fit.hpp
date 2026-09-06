// Closed-loop identification against the model the controller actually
// realizes.
//
// The conductor knows the gains it applied. For a position step with
// v_ref = 0 the controller commands
//
//     a_des = kx (p_ref - p) - kv p_dot
//
// the inner loop (attitude for x/y, thrust for z) delivers a fraction of
// that with an effective first-order lag, and the vehicle integrates it
// twice:
//
//     p_ddot = alpha * L(s) * a_des,   L(s) = 1 / (tau*s + 1)
//
// which closes to
//
//     P(s) = alpha*kx / (tau*s^3 + s^2 + alpha*kv*s + alpha*kx)      (1)
//
// with unit DC gain. Only TWO dynamic quantities are unknown:
//
//   alpha  the plant-gain factor -- thrust-map error, inner-loop droop.
//          1.0 means the vehicle produces exactly the acceleration the
//          controller asked for. This is the number the gain update needs.
//   tau    the effective lag of everything inside the loop (attitude
//          tracking, rate loop, transport). This is the number the
//          bandwidth ladder needs.
//
// Why this replaces the old (wn, zeta, delay) fit
// -----------------------------------------------
// That model was a free second-order system: three dynamic parameters for
// a plant that is third-order whenever tau is not small against the
// position loop. The optimizer could buy fit quality by raising wn,
// over-damping, and pushing dead time -- and since alpha was DERIVED as
// wn^2/kx, that trade landed directly in the estimate we act on. It
// needed a prior box on wn to stay physical, which made "wn pinned at the
// prior" a routine outcome; two pinned episodes then agree exactly, so
// they sail through the consistency gate that exists to catch failed
// identification.
//
// Where the transport delay belongs
// ----------------------------------
// tau is the lag of everything inside the loop, transport included. The
// sensing delay is not a harmless shift of the recorded curve: the
// controller feeds back that same delayed odometry, so the delay sits in
// the loop and costs phase exactly where stability is decided. Below the
// outer-loop bandwidth a delay and a lag are interchangeable to first
// order (e^-Td*s ~ 1/(1+Td*s) for Td*w << 1), so one effective lag
// carries both, and the ladder's stability margin then sees the delay it
// ought to see.
//
// Fitting a separate output shift as well was measured to be worse: on
// step responses simulated with an exact delay buffer, a free output
// shift roughly doubled the error in alpha (5.9% against 2.9%), because
// it gives the optimizer somewhere to put phase that actually belongs
// inside the loop.
//
// Here alpha and tau are separately identifiable because they enter (1)
// differently: alpha scales the loop gain (it moves the poles along the
// design locus set by kx, kv), while tau adds the third pole (it changes
// overshoot and ringing without moving the DC gain). The bounds below are
// numerical sanity only -- plausibility is judged afterwards, on an
// unconstrained estimate, so a parameter resting on a bound once again
// means the fit failed rather than that a prior did its job.
#ifndef GEO_TUNER__CORE__LOOP_FIT_HPP_
#define GEO_TUNER__CORE__LOOP_FIT_HPP_

#include <Eigen/Dense>

#include <cmath>

namespace geo_tuner
{

struct LoopFitResult
{
  double alpha{};       // plant-gain factor; 1.0 = the model is exact
  double tau{};         // effective in-loop lag [s]
  double delay{};       // residual output/measurement shift [s]
  double amplitude{};   // fitted steady state (~ step; DC gain is 1 by construction)
  double rmse{};        // residual RMS, same units as y
  double nrmse{};       // rmse / |step|
  double overshoot{};   // fraction of step, straight from the data
  bool converged{false};
  bool at_bounds{false};   // a parameter rested on a numerical sanity bound
  bool ambiguous{false};   // two near-equal minima disagreed about alpha

  /// Effective closed-loop natural frequency implied by the fit.
  double wn_effective(double kx) const {return std::sqrt(alpha * kx);}
  double zeta_effective(double kx, double kv) const
  {
    return alpha * kv / (2.0 * std::sqrt(alpha * kx));
  }

  /// Quality gate applied before an identification is allowed to move a
  /// gain. `ambiguous` is the replacement for the old "prefer the
  /// solution nearest the prior" tie-break: rather than quietly picking
  /// the convenient minimum, a genuinely degenerate fit is refused.
  bool ok() const
  {
    return converged && nrmse < 0.15 && !at_bounds && !ambiguous;
  }
};

/// Unit step response of (1), evaluated at t (t <= 0 gives 0).
///
/// Closed form via the residues of the three closed-loop poles, so a fit
/// iteration costs three exponentials per sample rather than a numerical
/// integration -- the conductor runs this inside its control timer.
Eigen::VectorXd closed_loop_step(
  const Eigen::VectorXd & t, double kx, double kv, double alpha, double tau);

/// Identify (alpha, tau, delay, amplitude) from one recorded step.
///
/// t: seconds, 0 at the step command. y: position relative to the
/// pre-step position, same sign convention as `step`.
/// kx, kv: the gains that were applied while the step was flown.
/// tau_guess: prior for the in-loop lag, e.g. attctrl_tau. Used only to
/// seed the multi-start; it does not constrain the answer.
/// model_output_delay: fit a separate output shift on top of the in-loop
/// lag. Off by default and it should stay off -- see "Where the transport
/// delay belongs" above. The switch exists so the regression test can
/// demonstrate the worse variant.
LoopFitResult fit_closed_loop(
  const Eigen::VectorXd & t, const Eigen::VectorXd & y, double step,
  double kx, double kv, double tau_guess = 0.15, bool model_output_delay = false);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__LOOP_FIT_HPP_
