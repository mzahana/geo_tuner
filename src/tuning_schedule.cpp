#include "geo_tuner/tuning_schedule.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace geo_tuner
{
namespace
{

std::string fmt(double v, int precision)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(precision);
  os << v;
  return os.str();
}

}  // namespace

double ramp_toward(double from, double to, double max_delta)
{
  const double d = to - from;
  if (max_delta <= 0.0 || std::abs(d) <= max_delta) {return to;}
  return from + (d > 0.0 ? max_delta : -max_delta);
}

std::array<double, 3> ramp_toward(
  const std::array<double, 3> & from, const std::array<double, 3> & to,
  double max_delta)
{
  const std::array<double, 3> d{to[0] - from[0], to[1] - from[1], to[2] - from[2]};
  const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  if (max_delta <= 0.0 || n <= max_delta || n < 1e-9) {return to;}
  const double k = max_delta / n;
  return {from[0] + k * d[0], from[1] + k * d[1], from[2] + k * d[2]};
}

double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

bool z_leg_clears_floor(
  double hover_agl, double leg_offset, double step_size_z, double margin,
  double floor)
{
  if (leg_offset >= 0.0) {return true;}   // an up leg never flies at the ground
  return hover_agl + leg_offset - step_size_z * std::max(0.0, margin) >= floor;
}

int axis_index(const std::string & axis)
{
  if (axis == "x") {return 0;}
  if (axis == "y") {return 1;}
  if (axis == "z") {return 2;}
  return -1;   // "yaw"
}

double TuningSchedule::yaw_of(const std::array<double, 4> & q)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double TuningSchedule::axis_step_mag() const
{
  const std::string ax = axes.empty() ? "x" : axes[axis_idx];
  if (ax == "yaw") {return yaw_step;}
  return ax == "z" ? step_size_z : step_size;
}

double TuningSchedule::next_leg(double mag) const
{
  if (!bidirectional) {
    return mag * step_sign;
  }
  return leg_offset != 0.0 ? 0.0 : mag * step_sign;
}

bool TuningSchedule::is_quiet(double now)
{
  if (!odom) {
    quiet_t0_.reset();
    return false;
  }
  const double mag = axis_step_mag();
  double err = 0.0;
  for (int i = 0; i < 3; ++i) {
    err = std::max(err, std::abs(odom->pos[i] - setpoint[i]));
  }
  const double speed = std::sqrt(
    odom->vel[0] * odom->vel[0] + odom->vel[1] * odom->vel[1] +
    odom->vel[2] * odom->vel[2]);
  const double tol_pos = std::min(settle_tol_pos, settle_tol_frac * mag);
  bool ok = err < tol_pos && speed < settle_tol_vel;
  if (ok && !axes.empty() && axes[axis_idx] == "yaw") {
    double dyaw = yaw_of(odom->quat) - setpoint_yaw;
    dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));
    const double tol_yaw = std::min(settle_tol_yaw, settle_tol_frac * mag);
    ok = std::abs(dyaw) < tol_yaw && std::abs(odom->body_rates[2]) < 3.0 * tol_yaw;
  }
  if (!ok) {
    quiet_t0_.reset();
    return false;
  }
  if (!quiet_t0_) {
    quiet_t0_ = now;
  }
  return (now - *quiet_t0_) >= settle_quiet_time;
}

bool TuningSchedule::response_settled(double t_end_abs) const
{
  const double t_end = t_end_abs - step_t0;
  const double w0 = t_end - episode_quiet_time;
  std::vector<double> ys;
  for (const auto & [t, y] : recording) {
    if (t >= w0) {ys.push_back(y);}
  }
  if (ys.size() < 5) {return false;}
  const double step = step_applied;
  const double band =
    std::max(episode_settle_band * std::abs(step), episode_settle_floor);
  const auto [lo, hi] = std::minmax_element(ys.begin(), ys.end());
  if (*hi - *lo > band) {return false;}
  double mean = 0.0;
  for (double y : ys) {mean += y;}
  mean /= static_cast<double>(ys.size());
  return mean * step > 0.0 && std::abs(mean) >= 0.6 * std::abs(step);
}

bool TuningSchedule::bucket_settled() const
{
  if (rep < min_episodes) {return false;}
  if (bucket.count() != rep) {return false;}
  const auto & vals = bucket.alphas;
  if (vals.size() < 2) {return false;}
  const auto [lo, hi] = std::minmax_element(vals.begin(), vals.end());
  return (*hi / *lo) <= early_stop_spread;
}

bool TuningSchedule::advance_axis(const std::array<double, 3> & hover, double hover_yaw)
{
  rep = 0;
  bucket = EpisodeBucket();
  leg_offset = 0.0;
  setpoint_yaw = hover_yaw;
  ++axis_idx;
  bool wrapped = false;
  if (axis_idx >= axes.size()) {
    axis_idx = 0;
    wrapped = true;
  }
  setpoint = hover;
  return wrapped;
}

double TuningSchedule::step_ceiling(double safety_max_pos_error) const
{
  return std::min(max_step_size, 0.8 * safety_max_pos_error);
}

std::pair<bool, std::string> TuningSchedule::validate_step(
  double value, bool is_yaw, double safety_max_pos_error) const
{
  if (!std::isfinite(value)) {
    return {false, "not a finite number"};
  }
  if (is_yaw) {
    if (!(value >= 0.05 && value <= max_yaw_step)) {
      return {false, "outside [0.05, " + fmt(max_yaw_step, 2) + "] rad (max_yaw_step)"};
    }
    return {true, ""};
  }
  const double ceiling = step_ceiling(safety_max_pos_error);
  if (value < min_step_size) {
    return {false, "below min_step_size " + fmt(min_step_size, 2) + " m"};
  }
  if (value > ceiling) {
    return {false,
      "above " + fmt(ceiling, 2) + " m, the largest safe step here (min of "
      "max_step_size " + fmt(max_step_size, 2) + " m and 0.8 x "
      "safety.max_pos_error " + fmt(safety_max_pos_error, 2) + " m -- a larger "
      "step trips the position-error abort the moment it is commanded)"};
  }
  if (value < small_step_warn) {
    return {true,
      "small: the response may not clear odometry noise, so the fit-quality "
      "gate (nrmse < 0.15) will reject episodes and the session may end "
      "'keeping gains'. Watch nrmse in the report."};
  }
  return {true, ""};
}

}  // namespace geo_tuner
