// In-flight auto-tuner for the mav_controllers_ros geometric controller.
//
// While the vehicle hovers in OFFBOARD under the geometric controller, this
// node flies the session in ROUNDS:
//
//   1. publishes hover setpoints (MultiDOFJointTrajectory, the controller's
//      standard setpoint input) and, per axis, a few position steps that
//      excite the loop; the whole time it records setpoint, velocity,
//      commanded acceleration (SE3Command) and the gains in force;
//   2. at the end of a round identifies each axis's acceleration loop
//      (plant gain + lag, with confidence intervals) from EVERYTHING recorded
//      so far -- core/accel_loop_id.hpp;
//   3. decides per axis (core/loop_design.hpp): gains the evidence supports
//      and with phase margin are CONFIRMED and left alone; otherwise the
//      lag-aware design is applied;
//   4. flies the next round on the applied gains and VALIDATES them on that
//      round's data alone; a failed validation restores the entry gains.
//
// Refuses to start while geometric_mavros's thrust-scale estimator is on: it
// adapts the very plant gain being measured.
//
// A fully independent SafetyMonitor watches odometry the entire time. Any
// violation aborts the session: gains are restored to the last known-safe
// set and the node holds a hover setpoint. The pilot's RC mode switch out
// of OFFBOARD always overrides everything -- this node never arms, disarms
// or changes flight modes.
//
// The node is intentionally plant-agnostic: it works identically against
// the lightweight simulator (geo_tuner quad_sim), PX4 SITL, and the real
// vehicle, because it only talks to the controller's ROS interface.
#ifndef GEO_TUNER__TUNING_CONDUCTOR_HPP_
#define GEO_TUNER__TUNING_CONDUCTOR_HPP_

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
#include <mavros_msgs/msg/state.hpp>
#endif

#include <mav_controllers_ros/msg/se3_command.hpp>

#include "geo_tuner/core/accel_loop_id.hpp"
#include "geo_tuner/core/aggregate.hpp"
#include "geo_tuner/core/loop_design.hpp"
#include "geo_tuner/core/safety.hpp"
#include "geo_tuner/core/session_recording.hpp"
#include "geo_tuner/tuning_schedule.hpp"

namespace geo_tuner
{

enum class TunerState
{
  WAIT_ODOM,
  WAIT_ENABLE,
  WAIT_OFFBOARD,
  TOO_LOW,
  GOTO_HOVER,
  SETTLE,
  STEP,
  ANALYZE,
  IDENTIFY,       // round flown; identification running off the control thread
  UPDATE_GAINS,
  DONE,
  ABORT,
};

const char * to_string(TunerState s);

/// States in which the conductor is actively flying the vehicle (safety
/// monitoring + offboard supervision apply).
bool is_active_state(TunerState s);

/// Per-axis session bookkeeping across rounds.
struct AxisProgress
{
  bool active{true};               // still flown in the next round
  bool pending_validation{false};  // gains applied last round, not yet validated
  bool validated{false};
  AccelLoopResult id_design;       // identification that designed the applied gains
  double yaw_T_design{0.0};        // yaw: T that designed the applied tau
  std::vector<double> overshoots;  // this round's measured overshoots
  std::vector<double> yaw_T;       // yaw: this round's accepted time constants
  AccelLoopResult id_last;         // latest identification, for the report
  std::string outcome;             // final one-liner for report/panel
  // What the session did to this axis, as one machine-readable word, because
  // `validated` alone reads as a failure on an axis that was confirmed and
  // never needed changing: confirmed | updated_validated | restored |
  // kept_inconclusive | kept_no_estimate | kept_update_unvalidated | unfinished.
  std::string result{"unfinished"};
};

class TuningConductor : public rclcpp::Node
{
public:
  explicit TuningConductor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using GetParameters = rcl_interfaces::srv::GetParameters;
  using SetParameters = rcl_interfaces::srv::SetParameters;
  using Trigger = std_srvs::srv::Trigger;
  using Gains = std::map<std::string, std::pair<double, double>>;  // axis -> (kx, kv)

  // ---- plumbing ----
  double now_s();
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void publish_setpoint();
  void status(const std::string & text, bool log = true);
  void goto_state(TunerState s);
  bool in_offboard() const;

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & params);

  // ---- gain get/set through the controller's parameter interface ----
  void request_gains(std::optional<double> now = std::nullopt);
  void apply_yaw_tau(double tau);
  void apply_gains(const Gains & gains, std::optional<double> yaw_tau = std::nullopt);

  // ---- state machine ----
  void tick();
  void st_wait_odom(double now);
  void st_wait_enable(double now);
  void st_wait_offboard(double now);
  void st_too_low(double now);
  void st_goto_hover(double now);
  void st_settle(double now);
  void st_step(double now);
  void st_analyze(double now);
  void st_update_gains(double now);
  void st_identify(double now);
  /// ABORT: confirm the gain restore, retrying until the controller accepts.
  void st_abort(double now);
  void send_restore(double now);

  void analyze_yaw();
  void dump_episode(
    const std::string & ax, const Eigen::VectorXd & t,
    const Eigen::VectorXd & y, double step);
  /// End of a round: identify, validate what the previous round applied,
  /// decide, apply. Moves to the next round or finishes.
  void finish_round();
  /// Second half of finish_round, once the identifications are in.
  void conclude_round();
  void decide_position_axis(const std::string & ax, Gains & to_apply);
  bool decide_yaw();   // true when a new tau was applied
  /// Re-enter the episode loop for the next repetition on this axis.
  void fly_next_rep();
  void episode_finished();
  void advance_axis();
  /// Move to an axis still active this round. Returns false when the round
  /// is over (finish_round has then taken over).
  bool seek_active_axis();
  void record_sample();
  void save_session_recording();
  void cmd_cb(const mav_controllers_ros::msg::SE3Command::SharedPtr msg);
  /// Thrust-estimator precondition, polled until it reads false.
  bool estimator_ok(double now);
  void start_next_round();
  void finish();
  void set_trim_diagnosis();
  /// Command-accel limits of the controller [m/s^2]: (lateral, vertical),
  /// 0 where unknown.
  std::pair<double, double> command_limits() const;
  /// Why a step of `step` m on a lateral (or vertical) axis would drive the
  /// command near its limit at the current gains; empty when it would not.
  std::string step_saturation_reason(double step, bool vertical) const;
  /// Non-empty (the reason) while either configured step would saturate.
  std::string saturation_gate() const;
  void abort(const std::string & reason);

  // ---- hover point and altitude ----

  /// Fix the point this session will tune about: in "capture" mode the
  /// vehicle's own position and heading at the moment it starts flying
  /// (so the handover is bumpless and the pilot chooses the spot), in
  /// "fixed" mode the hover_position/hover_yaw parameters.
  void capture_hover();

  /// Freeze the commanded setpoint where the vehicle is now. The only
  /// safe thing to command when something has gone wrong: it holds, and
  /// it can never be a large step.
  void hold_here();

  /// Height above ground [m], from the AGL topic when it is fresh, else
  /// odometry z minus the surveyed ground_z. Nullopt without odometry.
  std::optional<double> agl_now() const;

  /// Which of those two the number came from, for health/logs.
  const char * agl_source() const;

  /// AGL the vehicle would tune at: the hover point's height, obtained by
  /// shifting the current AGL by the hover point's offset in odometry z.
  std::optional<double> hover_agl() const;

  /// Height the vehicle must clear the safety floor by to fly a downward
  /// z step: the step itself plus the transient that undershoots past it.
  double z_step_clearance() const;

  /// Lowest AGL a session may tune at: the operator's min_tuning_altitude,
  /// or the floor plus the room a downward z step needs, whichever binds.
  double min_start_altitude() const;

  /// (ok, reason): may this session start tuning at the hover point?
  std::pair<bool, std::string> altitude_gate() const;

  /// Common entry into the flying phase from WAIT_ODOM / WAIT_OFFBOARD:
  /// captures the hover point and applies the altitude gate.
  void begin_flying();

  // ---- ground-station control ----
  void srv_start(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_abort(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_accept(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_restore_safe(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_reset(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);

  void publish_health();
  void write_report(const std::string & status);
  void freeze_session_end();

  // ---- interfaces ----
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr sp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr health_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr agl_sub_;
#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
#endif
  rclcpp::Subscription<mav_controllers_ros::msg::SE3Command>::SharedPtr cmd_sub_;
  rclcpp::Client<GetParameters>::SharedPtr get_param_cli_;
  // geometric_mavros node's parameters: the thrust-estimator precondition.
  rclcpp::Client<GetParameters>::SharedPtr estimator_param_cli_;
  rclcpp::Client<SetParameters>::SharedPtr set_param_cli_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, abort_srv_, accept_srv_,
    restore_srv_, reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_, health_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---- configuration ----
  double rate_hz_{50.0};
  // "capture" (default): tune about wherever the pilot hands the vehicle
  // over. "fixed": fly to hover_position first -- for simulators, which
  // have no pilot. hover_position/hover_yaw are read only in fixed mode.
  std::string hover_mode_{"capture"};
  std::vector<double> hover_{0.0, 0.0, 3.0};
  double hover_yaw_param_{0.0};
  // Speed the setpoint is allowed to travel at when it has to move to a
  // distant point. The controller sees a position error, not a
  // trajectory: a setpoint that teleports is a step command, and a large
  // one saturates the vehicle at max_accel until it arrives.
  double hover_speed_{0.7};        // m/s
  double hover_yaw_rate_{0.5};     // rad/s
  // The session refuses to start below this height above ground, and says
  // so instead of flying anywhere. Live-settable (RViz tuner panel).
  double min_tuning_altitude_{5.0};   // m AGL
  // Fallback AGL when no AGL topic is arriving: odometry z minus this.
  double ground_z_{0.0};
  double agl_timeout_{2.0};        // s before the AGL topic counts as gone
  // A z step does not stop at its commanded amplitude: the response
  // overshoots it. The clearance a downward leg needs above the safety
  // floor is the step times (1 + this).
  double z_step_margin_{0.5};
  double settle_time_{4.0};
  double hover_timeout_{20.0};
  double episode_time_{6.0};
  std::string episode_dump_dir_;   // per-episode CSVs for offline analysis; "" = off
  int episode_seq_{0};
  // Session data directory: when set, the report gets a timestamped name and
  // the episode CSVs a per-session subdirectory under it, both derived from
  // session_stamp_ (set when the baseline is captured).
  std::string output_dir_;
  std::string session_stamp_;
  double service_timeout_{5.0};
  double yaw_T_target_{0.35};
  double yaw_tau_min_{0.15};
  double yaw_tau_max_{1.2};
  // Per-episode nrmse screen for the yaw first-order fit. Looser than the
  // 0.15 the closed-loop fit uses: yaw moves little against odometry noise
  // and the bucket's median + consistency gate is the real protection.
  double yaw_fit_nrmse_{0.25};
  // Design and decision (core/loop_design.hpp). wn_target applies to x/y,
  // wn_target_z to z.
  DecisionConfig decision_;
  double wn_target_z_{1.6};
  double zeta_target_{0.95};
  double max_change_{1.6};
  int max_rounds_{3};
  double pm_tolerance_{5.0};       // deg: hysteresis on "gains in force keep the margins"
  double max_overshoot_{0.25};     // measured overshoot a validation accepts
  double yaw_tolerance_{0.25};     // |log(T/T_target)| <= log(1 + this) confirms yaw
  double consistency_{1.35};       // yaw episode agreement gate
  AccelLoopConfig id_cfg_;
  // Node whose enable_thrust_estimator must be false before a session may
  // start ("" skips the check -- simulators without geometric_mavros only).
  std::string estimator_node_;
  double min_settle_time_{0.3};
  bool adaptive_episode_{true};
  // How a position episode ends: "motion" (stopped, past the minimum
  // transient time) or "settled" (parked within a band of the step -- the
  // pre-2026-09-14 rule, kept for A/B runs).
  std::string episode_end_{"motion"};
  // Command-limit guard: a step whose predicted peak command kx*step exceeds
  // this fraction of the controller's limit is refused, and an episode whose
  // measured peak does is flagged. <= 0 disables.
  double saturation_margin_{0.8};
  double ctrl_max_accel_{0.0};     // m/s^2, 0 = unknown
  double ctrl_max_tilt_{0.0};      // rad, 0 = unknown
  double ep_u_peak_lat_{0.0};      // this episode's peak commanded accel [m/s^2]
  double ep_u_peak_z_{0.0};
  double min_episode_time_{2.0};
  double episode_settle_periods_{4.0};
  double hover_capture_radius_{0.3};
  double hover_capture_speed_{0.3};
  std::string report_path_{"/tmp/geo_tuner_report.yaml"};
  bool require_enable_{false};
  bool require_offboard_{true};
  std::string offboard_mode_{"OFFBOARD"};

  // ---- state ----
  /// The point this session tunes about, and the heading it holds there.
  /// Steps are offsets from these; nothing else is ever commanded.
  std::array<double, 3> hover_pt_{0.0, 0.0, 3.0};
  double hover_yaw_{0.0};
  bool hover_captured_{false};
  // GOTO_HOVER: the ramp's own travel time, added to hover_timeout so a
  // long (fixed-mode) approach is not reported as a failure to arrive.
  double goto_t0_{-1.0};
  double goto_eta_{0.0};
  std::optional<double> agl_msg_;
  double agl_msg_t_{0.0};
  bool agl_fresh_{false};
  double too_low_log_t_{0.0};
  TuningSchedule sched_;
  SafetyMonitor safety_;
  TunerState state_{TunerState::WAIT_ODOM};
  double state_t0_{0.0};
  std::optional<std::string> px4_mode_;
  std::optional<double> yaw_tau_;   // effective yaw time-constant param
  std::optional<double> attctrl_tau_;  // seeds the in-loop lag prior
  double pre_step_yaw_{0.0};
  double pre_step_pos_{0.0};
  size_t round_{0};                 // 0-based round index
  double episode_min_time_{0.0};    // earliest adaptive stop [s]
  // Where the session's wall time goes: the report carries the total and,
  // per episode, how long the pre-step settle and the recording took --
  // the numbers that say which knob to turn when a session felt long.
  double session_t0_{-1.0};         // first moment the session flew
  double last_settle_dur_{0.0};     // SETTLE time before the current episode
  std::optional<Gains> gains_;
  std::optional<Gains> safe_gains_;
  // Frozen at baseline capture; the "old" column of the panel's result view.
  // safe_gains_ won't do: it is promoted after every validated bucket.
  std::optional<Gains> baseline_gains_;
  std::optional<double> baseline_yaw_tau_;
  std::optional<double> safe_yaw_tau_;   // last validated (or entry) yawctrl_tau
  // Per-axis one-line outcome of the last finished bucket ("alpha 1.18" or
  // "kept: ..."), published in health for the panel's result view.
  std::map<std::string, std::string> session_result_;
  std::vector<YAML::Node> results_;
  std::string abort_reason_;
  std::string diagnosis_;
  // Acceleration feedforward trim [m/s^2]: cancels steady-state offsets
  // (thrust-map error on z, wind on x/y) the way an integral term would,
  // but transparently and bounded. Sent as the setpoint's acceleration,
  // which the controller uses as feedforward a_ref.
  std::array<double, 3> a_trim_{0.0, 0.0, 0.0};
  // What the session ended with, frozen when it completes or aborts. The
  // report is written again on accept, typically after landing: by then
  // leaving OFFBOARD has zeroed a_trim_ and the clock has run on (2026-09-14:
  // report said trim [0, 0, 0] and 358 s for a 313 s session).
  struct SessionEnd
  {
    double t{0.0};                       // node clock [s]
    std::array<double, 3> trim{};
    std::string stamp;                   // local wall time
  };
  std::optional<SessionEnd> session_end_;
  int trim_updates_{0};
  double trim_update_threshold_{0.05};   // m; SETTLE-phase learner gate (T4)
  std::map<std::string, AxisProgress> axis_prog_;
  // Round-end identification runs on a worker: a full session costs up to
  // seconds on the vehicle computer, and the 50 Hz tick (setpoints, odometry
  // staleness, safety) must not stall behind it.
  struct RoundIds
  {
    AccelLoopResult validation;   // this round only (gains applied last round)
    AccelLoopResult all;          // every round (the plant)
  };
  std::future<std::map<std::string, RoundIds>> id_future_;
  std::map<std::string, RoundIds> round_ids_;
  double max_identify_s_{30.0};
  // Gain restore after an abort: "", "pending", "confirmed", "UNCONFIRMED".
  std::string restore_state_;
  int restore_tries_{0};
  double restore_t0_{0.0};
  // True when an AGL topic is configured: required to start and throughout;
  // losing it mid-session must not silently re-base the floor on odometry z.
  bool agl_required_{false};
  // Session recording (identification input) and its segmentation.
  SessionRecording recording_;
  int rec_seg_{0};
  bool rec_gap_{true};
  size_t round_first_sample_{0};
  double controller_mass_{0.0};    // kg, read from the controller with the gains
  std::optional<std::array<double, 3>> last_cmd_force_;
  double last_cmd_t_{-1.0};
  // Estimator precondition.
  std::optional<rclcpp::Client<GetParameters>::SharedFuture> pending_est_;
  int64_t est_req_id_{0};
  double est_request_t0_{0.0};
  std::optional<bool> estimator_off_;
  std::string precondition_;       // why the session is refusing to start
  std::string session_csv_;        // where the session recording was saved
  bool finish_after_update_{false};  // last round's parameter write ends the session
  double max_trim_{3.0};            // m/s^2 per axis
  int max_trim_updates_{8};
  std::optional<rclcpp::Client<GetParameters>::SharedFuture> pending_get_;
  std::optional<rclcpp::Client<SetParameters>::SharedFuture> pending_set_;
  int64_t get_req_id_{0};
  double request_t0_{0.0};          // when the baseline read was issued
  int request_tries_{0};
  bool waiting_logged_{false};
  bool baseline_read_{false};
  bool start_requested_{true};
};

}  // namespace geo_tuner

#endif  // GEO_TUNER__TUNING_CONDUCTOR_HPP_
