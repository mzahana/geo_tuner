#include "geo_tuner/core/step_fit.hpp"

#include <cmath>

namespace geo_tuner
{
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

}  // namespace geo_tuner
