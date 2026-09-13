// The session recording: what the acceleration-loop identification needs,
// sampled at the conductor's tick rate while it flies the vehicle.
//
// One format for three producers/consumers -- the conductor writes it, the
// bag exporter (scripts/geo-tuner-bag-export) writes it for older flights,
// and geo-tuner-identify and the tests read it -- so an offline verdict is
// computed from exactly the data the flight decided on.
//
// CSV columns: t,seg,rx,ry,rz,vx,vy,vz,ux,uy,uz,kxx,kxy,kxz,kvx,kvy,kvz
#ifndef GEO_TUNER__CORE__SESSION_RECORDING_HPP_
#define GEO_TUNER__CORE__SESSION_RECORDING_HPP_

#include <array>
#include <string>
#include <vector>

#include "geo_tuner/core/accel_loop_id.hpp"

namespace geo_tuner
{

struct SessionSample
{
  double t{0.0};
  int seg{0};                         // contiguous-segment id
  std::array<double, 3> r{};          // setpoint position
  std::array<double, 3> v{};          // measured velocity
  std::array<double, 3> u{};          // commanded acceleration, gravity removed
  std::array<double, 3> kx{}, kv{};   // gains in force
};

class SessionRecording
{
public:
  std::vector<SessionSample> samples;

  void clear() {samples.clear();}
  size_t size() const {return samples.size();}

  /// Per-axis segments (axis 0..2) of the samples with index >= first.
  std::vector<AxisSegment> segments(int axis, size_t first = 0) const;

  static const char * csv_header();
  static std::string csv_line(const SessionSample & s);
  bool save_csv(const std::string & path) const;
  /// Throws std::runtime_error on an unreadable or malformed file.
  static SessionRecording load_csv(const std::string & path);
};

}  // namespace geo_tuner

#endif  // GEO_TUNER__CORE__SESSION_RECORDING_HPP_
