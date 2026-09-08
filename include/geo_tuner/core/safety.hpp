// Independent safety monitor for in-flight tuning episodes.
//
// Pure logic (no ROS) so it is unit-testable. The conductor node feeds it
// odometry samples; any violation aborts the tuning session into a hover
// hold with the last known-safe gains.
#ifndef GEO_TUNER__CORE__SAFETY_HPP_
#define GEO_TUNER__CORE__SAFETY_HPP_

#include <array>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace geo_tuner
{

enum class Violation
{
  TILT,
  POS_ERROR,
  VELOCITY,
  ALTITUDE_LOW,
  ALTITUDE_HIGH,
  OSCILLATION,
  ODOM_STALE,
};

/// The human-readable text carried into the abort reason.
const char * to_string(Violation v);

struct SafetyLimits
{
  double max_tilt{0.6};          // rad (~34 deg), > controller max_tilt_angle
  double max_pos_error{2.0};     // m, distance from active setpoint
  double max_velocity{4.0};      // m/s
  // Altitude limits are in metres above ground (OdomSample::agl when the
  // node has an AGL source, otherwise the odometry z, which is only AGL
  // when the local frame's origin happens to sit on the ground -- it did
  // not in the field, by 6.6 m).
  double min_altitude{1.0};      // m AGL
  double max_altitude{40.0};     // m AGL
  double odom_timeout{0.3};      // s
  // Oscillation detector: RMS of mean-removed body rates over the window
  double osc_window{2.0};        // s
  double osc_rate_rms{1.2};      // rad/s, roll+pitch combined
  double osc_yaw_rate_rms{1.5};  // rad/s, yaw alone (drag-torque axis)
  int osc_min_samples{20};
};

struct OdomSample
{
  double t{};                              // s, receiver clock (staleness checks)
  double t_stamp{};                        // s, sensor clock (episode time axis)
  std::array<double, 3> pos{};             // (x, y, z) m
  std::array<double, 3> vel{};             // (vx, vy, vz) m/s
  std::array<double, 4> quat{1, 0, 0, 0};  // (w, x, y, z)
  std::array<double, 3> body_rates{};      // rad/s
  // Height above ground [m], when the node has a source for it
  // (mavros/global_position/rel_alt, or odometry z minus a surveyed
  // ground_z). Unset falls back to pos[2] for the altitude limits.
  std::optional<double> agl{};
};

/// Angle between body z and world z, from a (w,x,y,z) quaternion.
double tilt_from_quat(const std::array<double, 4> & q);

class SafetyMonitor
{
public:
  SafetyMonitor() = default;
  explicit SafetyMonitor(const SafetyLimits & limits)
  : limits_(limits) {}

  /// Feed one odometry sample; returns violations (empty = safe).
  ///
  /// check_min_altitude is cleared by the one caller that is deliberately
  /// sitting below the floor -- the conductor holding position after it
  /// refused to start too low. Everything else still applies there.
  std::vector<Violation> check(
    const OdomSample & s, const std::optional<std::array<double, 3>> & setpoint,
    bool check_min_altitude = true);

  /// Height the altitude limits are judged against.
  static double altitude_of(const OdomSample & s) {return s.agl.value_or(s.pos[2]);}

  std::vector<Violation> check_stale(double now) const;

  void reset();

  const SafetyLimits & limits() const {return limits_;}
  SafetyLimits & limits() {return limits_;}

private:
  SafetyLimits limits_;
  std::deque<std::array<double, 4>> rates_;  // (t, wx, wy, wz)
  std::optional<double> last_t_;
};

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__SAFETY_HPP_
