// Pure episode-scheduling logic for the tuning conductor.
//
// Split out of the node so that the decisions which shorten a session --
// the bidirectional leg schedule, the settle predicate, the adaptive
// episode end, the sequential bucket stop and the step-amplitude
// validation -- are exercised by unit tests with no ROS context and no
// vehicle. The node owns one of these and calls into it; it holds no
// publishers, no clock and no parameters of its own.
#ifndef GEO_TUNER__TUNING_SCHEDULE_HPP_
#define GEO_TUNER__TUNING_SCHEDULE_HPP_

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geo_tuner/core/aggregate.hpp"
#include "geo_tuner/core/safety.hpp"

namespace geo_tuner
{

/// Index of an axis name in a position triple; -1 for "yaw".
int axis_index(const std::string & axis);

/// One step of a rate-limited approach: move `from` toward `to` by at most
/// `max_delta`, never overshooting. A setpoint that jumps is a step command
/// to the controller, and a large one saturates it -- the conductor
/// therefore never assigns a distant target directly, it ramps to it.
double ramp_toward(double from, double to, double max_delta);

/// Same, on a position triple, keeping the direction of travel: the whole
/// vector moves at one speed rather than each axis independently.
std::array<double, 3> ramp_toward(
  const std::array<double, 3> & from, const std::array<double, 3> & to,
  double max_delta);

/// Shortest signed difference between two angles [rad], wrapped to +/-pi.
double wrap_angle(double a);

/// Is the (axis, rung) bucket flown in this session? With
/// yaw_final_rung_only, yaw buckets fly only on the LAST rung: the yaw
/// target (a time constant) does not ladder with wn, so identifying it on
/// every rung re-measures the same thing -- the 2026-09-09 field session
/// spent six episodes on yaw across two rungs for one tau update's worth
/// of information.
bool axis_eligible(
  const std::string & axis, size_t rung, size_t n_rungs,
  bool yaw_final_rung_only);

/// (steps_done, steps_total) across the whole session, for a ground
/// station's progress display. Every eligible bucket counts at its planned
/// episodes_per_rung; a bucket that closed early counts as complete the
/// moment the schedule moves past it, so the fraction only ever moves
/// forward. (rung, axis_idx, rep) is the schedule's current position.
std::pair<int, int> session_progress(
  const std::vector<std::string> & axes, size_t n_rungs, int episodes_per_rung,
  size_t rung, size_t axis_idx, int rep, bool yaw_final_rung_only);

/// Would a vertical leg to `leg_offset` (m, signed, from the hover point)
/// keep clear of the altitude floor?
///
/// The commanded point is hover_agl + leg_offset, and the response
/// undershoots it by margin * step_size before it settles. Both terms scale
/// with the step amplitude, which is a live parameter -- so this moves
/// whenever the operator changes the vertical step size.
bool z_leg_clears_floor(
  double hover_agl, double leg_offset, double step_size_z, double margin,
  double floor);

class TuningSchedule
{
public:
  // ---- configuration (set from node parameters) ----
  bool bidirectional{true};
  double step_size{0.5};
  double step_size_z{0.4};
  double yaw_step{0.5};
  double max_step_size{2.0};
  double min_step_size{0.05};
  double max_yaw_step{1.0};
  double small_step_warn{0.25};
  double settle_tol_pos{0.06};
  double settle_tol_vel{0.10};
  double settle_tol_yaw{0.05};
  double settle_tol_frac{0.15};
  double settle_quiet_time{0.4};
  double episode_quiet_time{0.5};
  double episode_settle_band{0.04};
  double episode_settle_floor{0.01};
  int episodes_per_rung{3};
  int min_episodes{2};
  double early_stop_spread{1.15};

  // ---- live session state ----
  std::vector<std::string> axes{"z", "x", "y", "yaw"};
  size_t axis_idx{0};
  double step_sign{1.0};
  /// Commanded offset from the hover point on the active axis (m, or rad
  /// for yaw). The bidirectional schedule alternates it between 0 and
  /// +/- the step size; each transition is one episode.
  double leg_offset{0.0};
  double step_applied{0.0};   // signed step of the episode in flight
  std::array<double, 3> setpoint{0.0, 0.0, 3.0};
  double setpoint_yaw{0.0};
  int rep{0};
  EpisodeBucket bucket;
  std::vector<std::pair<double, double>> recording;   // (t, pos[axis])
  double step_t0{0.0};
  std::optional<OdomSample> odom;

  // ---- decisions ----

  /// Amplitude of the step this axis flies, in the axis's units.
  double axis_step_mag() const;

  /// Commanded offset for the next episode.
  ///
  /// Bidirectional: 0 -> +mag -> 0 -> -mag -> 0 ... so the return leg is
  /// itself a recorded step instead of dead flight time. Otherwise the
  /// classic schedule: always step out from the hover point.
  double next_leg(double mag) const;

  /// Vehicle settled on the current setpoint for settle_quiet_time.
  ///
  /// Tighter than the GOTO_HOVER capture gate: the step must start from
  /// a genuinely quiet state or the fit sees a superimposed transient.
  bool is_quiet(double now);

  /// The recorded response has reached and held its steady state.
  ///
  /// Recording past that point adds only flat samples: they carry no
  /// information about (wn, zeta, delay) and just cost flight time. The
  /// window must sit at the step's steady state (not at the flat piece
  /// during the transport delay), hence the amplitude check.
  bool response_settled(double t_end_abs) const;

  /// Enough consistent evidence to stop this bucket early.
  ///
  /// Requires (a) the minimum number of reps flown, (b) every one of them
  /// accepted -- a discard means the identification is already noisy, so
  /// fly the full bucket -- and (c) agreement tighter than the
  /// consistency gate the full bucket would have to pass.
  bool bucket_settled() const;

  /// Largest linear step that cannot trip the safety monitor.
  ///
  /// A step commands the setpoint to jump by the full amplitude while the
  /// vehicle is still at the old point, so the position error is
  /// momentarily equal to the step. At max_pos_error the monitor would
  /// abort the session the instant the step is issued; keep a margin.
  double step_ceiling(double safety_max_pos_error) const;

  /// (ok, note): note is a rejection reason, or a warning when ok.
  std::pair<bool, std::string> validate_step(
    double value, bool is_yaw, double safety_max_pos_error) const;

  /// Move to the next axis, clearing the leg schedule so a new axis
  /// always starts its steps from the hover point. Returns true when the
  /// axis list wrapped -- the caller then steps the wn ladder.
  ///
  /// hover_yaw is the heading the session was handed over at: yaw legs are
  /// offsets from it, never from an absolute zero (which would command a
  /// snap to north the moment the session starts).
  bool advance_axis(const std::array<double, 3> & hover, double hover_yaw);

  void reset_quiet() {quiet_t0_.reset();}

  static double yaw_of(const std::array<double, 4> & q);

private:
  std::optional<double> quiet_t0_;   // SETTLE quiet-window start
};

}  // namespace geo_tuner

#endif  // GEO_TUNER__TUNING_SCHEDULE_HPP_
