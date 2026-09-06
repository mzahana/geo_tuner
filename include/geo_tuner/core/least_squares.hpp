// Bounded nonlinear least squares.
//
// Stands in for scipy.optimize.least_squares(..., bounds=(lb, ub),
// method="trf"), which the Python implementation of the step fits used.
// The models fitted here are small (3-4 parameters), smooth and cheap to
// evaluate, so a projected Levenberg-Marquardt with a finite-difference
// Jacobian reproduces the same minima without pulling in a solver
// library. As in TRF, a parameter that wants to leave the box is held on
// the boundary -- which is what the fits' `at_bounds` gate inspects.
#ifndef GEO_TUNER__CORE__LEAST_SQUARES_HPP_
#define GEO_TUNER__CORE__LEAST_SQUARES_HPP_

#include <Eigen/Dense>
#include <functional>

namespace geo_tuner
{

struct LeastSquaresOptions
{
  double ftol{1e-8};      // relative reduction of the cost
  double xtol{1e-8};      // relative change of the parameter vector
  double gtol{1e-8};      // norm of the projected gradient
  int max_iter{200};
  double lambda_init{1e-3};
};

struct LeastSquaresResult
{
  Eigen::VectorXd x;      // solution
  Eigen::VectorXd fun;    // residuals at the solution
  double cost{};          // 0.5 * ||fun||^2
  bool success{false};    // a termination criterion was met (not iteration cap)
  int n_iter{0};
};

using ResidualFn = std::function<Eigen::VectorXd (const Eigen::VectorXd &)>;

/// Minimize 0.5*||f(x)||^2 subject to lb <= x <= ub, starting from x0
/// (which is clamped into the box first).
LeastSquaresResult least_squares_bounded(
  const ResidualFn & residual,
  const Eigen::VectorXd & x0,
  const Eigen::VectorXd & lb,
  const Eigen::VectorXd & ub,
  const LeastSquaresOptions & opts = {});

/// Elementwise clamp into [lb, ub] (numpy.clip on a vector).
Eigen::VectorXd clip(
  const Eigen::VectorXd & x, const Eigen::VectorXd & lb, const Eigen::VectorXd & ub);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__LEAST_SQUARES_HPP_
