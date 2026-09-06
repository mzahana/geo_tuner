#include "geo_tuner/core/least_squares.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geo_tuner
{
namespace
{

/// Forward (or backward, next to the upper bound) difference Jacobian.
Eigen::MatrixXd finite_difference_jacobian(
  const ResidualFn & residual, const Eigen::VectorXd & x, const Eigen::VectorXd & f,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub)
{
  const Eigen::Index n = x.size();
  const Eigen::Index m = f.size();
  Eigen::MatrixXd J(m, n);
  const double rel = std::sqrt(std::numeric_limits<double>::epsilon());
  for (Eigen::Index j = 0; j < n; ++j) {
    double h = rel * std::max(1.0, std::abs(x[j]));
    // Never step outside the box: the model may be undefined there.
    double sign = 1.0;
    if (x[j] + h > ub[j]) {
      sign = -1.0;
      if (x[j] - h < lb[j]) {
        h = 0.5 * std::max(ub[j] - lb[j], 0.0);
        if (h <= 0.0) {
          J.col(j).setZero();
          continue;
        }
      }
    }
    Eigen::VectorXd xp = x;
    xp[j] += sign * h;
    J.col(j) = (residual(xp) - f) / (sign * h);
  }
  return J;
}

}  // namespace

Eigen::VectorXd clip(
  const Eigen::VectorXd & x, const Eigen::VectorXd & lb, const Eigen::VectorXd & ub)
{
  return x.cwiseMax(lb).cwiseMin(ub);
}

LeastSquaresResult least_squares_bounded(
  const ResidualFn & residual, const Eigen::VectorXd & x0,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub,
  const LeastSquaresOptions & opts)
{
  LeastSquaresResult out;
  Eigen::VectorXd x = clip(x0, lb, ub);
  Eigen::VectorXd f = residual(x);
  double cost = 0.5 * f.squaredNorm();
  double lambda = opts.lambda_init;

  for (int it = 0; it < opts.max_iter; ++it) {
    out.n_iter = it + 1;
    const Eigen::MatrixXd J = finite_difference_jacobian(residual, x, f, lb, ub);
    const Eigen::VectorXd g = J.transpose() * f;

    // Projected gradient: a component pushing a parameter further out of
    // the box is not a descent direction, so it does not count.
    Eigen::VectorXd gp = g;
    for (Eigen::Index j = 0; j < x.size(); ++j) {
      const bool at_lo = x[j] <= lb[j] && g[j] > 0.0;
      const bool at_hi = x[j] >= ub[j] && g[j] < 0.0;
      if (at_lo || at_hi) {gp[j] = 0.0;}
    }
    if (gp.lpNorm<Eigen::Infinity>() < opts.gtol) {
      out.success = true;
      break;
    }

    const Eigen::MatrixXd JtJ = J.transpose() * J;
    Eigen::VectorXd diag = JtJ.diagonal();
    for (Eigen::Index j = 0; j < diag.size(); ++j) {
      if (!(diag[j] > 0.0)) {diag[j] = 1.0;}
    }

    bool accepted = false;
    bool terminate = false;
    for (int inner = 0; inner < 30; ++inner) {
      const Eigen::MatrixXd A = JtJ + lambda * diag.asDiagonal().toDenseMatrix();
      const Eigen::VectorXd dx = A.ldlt().solve(-g);
      if (!dx.allFinite()) {
        lambda *= 10.0;
        continue;
      }
      const Eigen::VectorXd x_new = clip(x + dx, lb, ub);
      const Eigen::VectorXd step = x_new - x;
      const Eigen::VectorXd f_new = residual(x_new);
      const double cost_new = 0.5 * f_new.squaredNorm();
      if (cost_new < cost) {
        const double dcost = cost - cost_new;
        const double dx_norm = step.norm();
        x = x_new;
        f = f_new;
        cost = cost_new;
        lambda = std::max(lambda / 3.0, 1e-12);
        accepted = true;
        if (dcost < opts.ftol * std::max(cost, 1e-300) ||
          dx_norm < opts.xtol * (opts.xtol + x.norm()))
        {
          terminate = true;
        }
        break;
      }
      lambda *= 3.0;
      if (lambda > 1e12) {
        // No downhill step exists inside the box: this is a minimum.
        terminate = true;
        break;
      }
    }
    if (terminate) {
      out.success = true;
      break;
    }
    if (!accepted) {
      out.success = true;   // exhausted the damping search at a minimum
      break;
    }
  }

  out.x = x;
  out.fun = f;
  out.cost = cost;
  return out;
}

}  // namespace geo_tuner
