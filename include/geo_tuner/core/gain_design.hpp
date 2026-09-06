// Principled gain design for mav_controllers_ros GeometricAttitudeControl.
//
// The outer (position) loop of the geometric controller is
// feedback-linearized:
//
//     a_fb = kx * e_pos + kv * e_vel   (+ integral, + feedforward)
//
// so the ideal closed loop per axis is a double integrator under PD control:
//
//     e_ddot + kv * e_dot + kx * e = 0
//
// which maps directly to second-order dynamics:
//
//     kx = wn^2          [1/s^2]
//     kv = 2 * zeta * wn [1/s]
//
// Gains are therefore designed by choosing (wn, zeta) subject to physical
// constraints (inner-loop bandwidth, latency, thrust headroom) instead of
// hand-twiddling kx/kv.
#ifndef GEO_TUNER__CORE__GAIN_DESIGN_HPP_
#define GEO_TUNER__CORE__GAIN_DESIGN_HPP_

#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace geo_tuner
{

inline constexpr double kGravity = 9.81;

/// Physical vehicle description (the only field-measured inputs).
struct VehicleParams
{
  double mass{};                  // kg, with battery
  double hover_throttle{};        // normalized [0..1], from PX4 log at flight voltage
  double max_tilt_angle{0.52};    // rad (30 deg default while tuning)

  /// Total max collective thrust [N] implied by the hover point.
  ///
  /// This is the value the geometric_mavros node must use so that
  /// commanded force in N maps to the correct normalized PX4 thrust.
  double max_thrust() const {return mass * kGravity / hover_throttle;}

  /// Max upward acceleration [m/s^2] beyond gravity compensation.
  double vertical_accel_headroom() const
  {
    return kGravity * (1.0 / hover_throttle - 1.0);
  }

  /// Max horizontal acceleration [m/s^2] at the tilt limit.
  double lateral_accel_max() const {return kGravity * std::tan(max_tilt_angle);}
};

/// Design targets and constraints for the cascaded loops.
struct LoopShape
{
  double attctrl_tau{0.3};            // controller param; attitude BW ~ 2/tau [rad/s]
  double timescale_separation{4.0};   // attitude BW / position BW, >= 3
  double latency{0.08};               // s, EKF + mavros + offboard round trip
  double latency_margin{0.35};        // require wn * latency <= this
  double zeta{0.95};                  // target damping (near-critical for chase)
  double z_gain_factor{1.6};          // z loop can be stiffer (direct thrust authority)

  double attitude_bandwidth() const {return 2.0 / attctrl_tau;}
  double wn_max_separation() const {return attitude_bandwidth() / timescale_separation;}
  double wn_max_latency() const {return latency_margin / latency;}
};

/// A complete, ready-to-apply geometric controller gain set.
struct GainSet
{
  std::array<double, 3> kx{};   // (x, y, z)
  std::array<double, 3> kv{};
  double wn_xy{};
  double wn_z{};
  double zeta{};
  double attctrl_tau{};
  double max_thrust{};
  std::vector<std::string> notes;
};

/// Map second-order targets to controller gains: (kx, kv).
std::pair<double, double> pd_from_wn_zeta(double wn, double zeta);

/// Inverse map: (wn, zeta) implied by a gain pair. Throws if kx <= 0.
std::pair<double, double> wn_zeta_from_pd(double kx, double kv);

/// Compute a gain set honoring all constraints.
///
/// wn_request: desired position-loop natural frequency [rad/s]. If unset,
/// the maximum allowed by the constraints is used. If the request exceeds
/// a constraint, it is clipped and a note is recorded.
GainSet design_gains(
  const VehicleParams & vehicle, const LoopShape & shape,
  std::optional<double> wn_request = std::nullopt);

/// One iteration of identification-based gain correction.
///
/// If the measured closed-loop natural frequency differs from the one
/// predicted by the applied gains, the discrepancy is a multiplicative
/// plant-gain error alpha (thrust-map error, inner-loop droop, ...):
///
///     effective dynamics: e_ddot = -alpha*(kx e + kv e_dot)
///     =>  wn_measured^2 = alpha * kx_applied
///
/// Solving for the gains that place the *effective* poles at the target:
///
///     kx_new = wn_target^2 / alpha
///     kv_new = 2 * zeta_target * wn_target / alpha
struct GainCorrection
{
  double kx_new{};
  double kv_new{};
  double alpha{};
};
GainCorrection correct_gains_from_identification(
  double kx_applied, double wn_measured, double wn_target, double zeta_target);

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__GAIN_DESIGN_HPP_
