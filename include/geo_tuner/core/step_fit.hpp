// Step response of a standard second-order system.
//
// This used to carry a (wn, zeta, delay) estimator that the conductor ran
// on every position episode. It was replaced by core/loop_fit, which
// identifies the closed loop against the gains that were actually
// applied; see that header for why a free second-order fit was the wrong
// model. What survives here is the shape itself, used as the zero-lag
// limit of the closed-loop model and to synthesise test data.
#ifndef GEO_TUNER__CORE__STEP_FIT_HPP_
#define GEO_TUNER__CORE__STEP_FIT_HPP_

#include <Eigen/Dense>

namespace geo_tuner
{

/// Unit step response of wn^2 / (s^2 + 2 zeta wn s + wn^2), for t >= 0.
Eigen::VectorXd second_order_step(const Eigen::VectorXd & t, double wn, double zeta);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__STEP_FIT_HPP_
