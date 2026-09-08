#include "geo_tuner/core/safety.hpp"

#include <algorithm>
#include <cmath>

namespace geo_tuner
{

const char * to_string(Violation v)
{
  switch (v) {
    case Violation::TILT: return "tilt limit exceeded";
    case Violation::POS_ERROR: return "position error bound exceeded";
    case Violation::VELOCITY: return "velocity bound exceeded";
    case Violation::ALTITUDE_LOW: return "below minimum altitude";
    case Violation::ALTITUDE_HIGH: return "above maximum altitude";
    case Violation::OSCILLATION: return "oscillation detected (body rate energy)";
    case Violation::ODOM_STALE: return "odometry stale";
  }
  return "unknown violation";
}

double tilt_from_quat(const std::array<double, 4> & q)
{
  const double x = q[1], y = q[2];
  // R[2][2] of the rotation matrix
  const double r22 = 1.0 - 2.0 * (x * x + y * y);
  return std::acos(std::clamp(r22, -1.0, 1.0));
}

std::vector<Violation> SafetyMonitor::check(
  const OdomSample & s, const std::optional<std::array<double, 3>> & setpoint,
  bool check_min_altitude)
{
  std::vector<Violation> v;
  last_t_ = s.t;

  if (tilt_from_quat(s.quat) > limits_.max_tilt) {
    v.push_back(Violation::TILT);
  }

  const double speed = std::sqrt(
    s.vel[0] * s.vel[0] + s.vel[1] * s.vel[1] + s.vel[2] * s.vel[2]);
  if (speed > limits_.max_velocity) {
    v.push_back(Violation::VELOCITY);
  }

  const double alt = altitude_of(s);
  if (check_min_altitude && alt < limits_.min_altitude) {
    v.push_back(Violation::ALTITUDE_LOW);
  }
  if (alt > limits_.max_altitude) {
    v.push_back(Violation::ALTITUDE_HIGH);
  }

  if (setpoint) {
    double sq = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double d = s.pos[i] - (*setpoint)[i];
      sq += d * d;
    }
    if (std::sqrt(sq) > limits_.max_pos_error) {
      v.push_back(Violation::POS_ERROR);
    }
  }

  // Rolling body-rate oscillation energy
  rates_.push_back({s.t, s.body_rates[0], s.body_rates[1], s.body_rates[2]});
  const double t0 = s.t - limits_.osc_window;
  while (!rates_.empty() && rates_.front()[0] < t0) {
    rates_.pop_front();
  }
  if (static_cast<int>(rates_.size()) >= limits_.osc_min_samples) {
    const double n = static_cast<double>(rates_.size());
    auto var = [this, n](int axis) {
        double mean = 0.0;
        for (const auto & r : rates_) {mean += r[axis];}
        mean /= n;
        double acc = 0.0;
        for (const auto & r : rates_) {
          const double d = r[axis] - mean;
          acc += d * d;
        }
        return acc / n;
      };
    if (std::sqrt(var(1) + var(2)) > limits_.osc_rate_rms) {
      v.push_back(Violation::OSCILLATION);
    }
    if (std::sqrt(var(3)) > limits_.osc_yaw_rate_rms) {
      v.push_back(Violation::OSCILLATION);
    }
  }

  return v;
}

std::vector<Violation> SafetyMonitor::check_stale(double now) const
{
  if (!last_t_) {return {};}
  if (now - *last_t_ > limits_.odom_timeout) {
    return {Violation::ODOM_STALE};
  }
  return {};
}

void SafetyMonitor::reset()
{
  rates_.clear();
  last_t_.reset();
}

}  // namespace geo_tuner
