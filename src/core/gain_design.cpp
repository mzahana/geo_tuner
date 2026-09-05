#include "geo_tuner/core/gain_design.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace geo_tuner
{
namespace
{

/// Match Python's round(x, n) closely enough for report/YAML output.
double round_to(double v, int digits)
{
  const double f = std::pow(10.0, digits);
  return std::round(v * f) / f;
}

std::string fmt(double v, int precision)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(precision);
  os << v;
  return os.str();
}

}  // namespace

std::pair<double, double> pd_from_wn_zeta(double wn, double zeta)
{
  return {wn * wn, 2.0 * zeta * wn};
}

std::pair<double, double> wn_zeta_from_pd(double kx, double kv)
{
  if (kx <= 0.0) {
    throw std::invalid_argument("kx must be > 0, got " + fmt(kx, 6));
  }
  const double wn = std::sqrt(kx);
  return {wn, kv / (2.0 * wn)};
}

GainSet design_gains(
  const VehicleParams & vehicle, const LoopShape & shape,
  std::optional<double> wn_request)
{
  std::vector<std::string> notes;
  const double wn_cap = std::min(shape.wn_max_separation(), shape.wn_max_latency());
  if (shape.wn_max_latency() < shape.wn_max_separation()) {
    notes.push_back(
      "Position bandwidth limited by latency (" + fmt(shape.latency * 1e3, 0) +
      " ms): wn <= " + fmt(shape.wn_max_latency(), 2) + " rad/s");
  } else {
    notes.push_back(
      "Position bandwidth limited by attitude loop (tau=" + fmt(shape.attctrl_tau, 6) +
      "): wn <= " + fmt(shape.wn_max_separation(), 2) + " rad/s");
  }

  const double wn_xy = wn_request ? std::min(*wn_request, wn_cap) : wn_cap;
  if (wn_request && *wn_request > wn_cap) {
    notes.push_back(
      "Requested wn=" + fmt(*wn_request, 2) + " clipped to " + fmt(wn_cap, 2) + " rad/s");
  }

  const double wn_z = std::min(
    wn_xy * std::sqrt(shape.z_gain_factor), wn_cap * std::sqrt(shape.z_gain_factor));

  const auto [kx_xy, kv_xy] = pd_from_wn_zeta(wn_xy, shape.zeta);
  const auto [kx_z, kv_z] = pd_from_wn_zeta(wn_z, shape.zeta);

  // Sanity: acceleration a 1 m step would command vs. physical headroom.
  const double a_step = kx_xy * 1.0;
  if (a_step > vehicle.lateral_accel_max()) {
    notes.push_back(
      "A 1 m lateral step commands " + fmt(a_step, 1) + " m/s^2 > tilt-limited " +
      fmt(vehicle.lateral_accel_max(), 1) + " m/s^2; the max_tilt/max_accel clamps "
      "will engage on large steps (acceptable, but keep field test steps small).");
  }
  if (vehicle.hover_throttle > 0.6) {
    notes.push_back(
      "Hover throttle " + fmt(vehicle.hover_throttle, 2) + " > 0.6: low thrust "
      "headroom; vehicle is heavy for its powertrain.");
  }

  GainSet g;
  g.kx = {round_to(kx_xy, 3), round_to(kx_xy, 3), round_to(kx_z, 3)};
  g.kv = {round_to(kv_xy, 3), round_to(kv_xy, 3), round_to(kv_z, 3)};
  g.wn_xy = wn_xy;
  g.wn_z = wn_z;
  g.zeta = shape.zeta;
  g.attctrl_tau = shape.attctrl_tau;
  g.max_thrust = round_to(vehicle.max_thrust(), 2);
  g.notes = std::move(notes);
  return g;
}

GainCorrection correct_gains_from_identification(
  double kx_applied, double wn_measured, double wn_target, double zeta_target)
{
  if (wn_measured <= 0.0 || kx_applied <= 0.0) {
    throw std::invalid_argument("wn_measured and kx_applied must be > 0");
  }
  GainCorrection c;
  c.alpha = (wn_measured * wn_measured) / kx_applied;
  c.kx_new = wn_target * wn_target / c.alpha;
  c.kv_new = 2.0 * zeta_target * wn_target / c.alpha;
  return c;
}

}  // namespace geo_tuner
