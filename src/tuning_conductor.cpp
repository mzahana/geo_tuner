#include "geo_tuner/tuning_conductor.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include "geo_tuner/core/aggregate.hpp"
#include "geo_tuner/core/first_order_fit.hpp"
#include "geo_tuner/core/gain_design.hpp"
#include "geo_tuner/core/loop_fit.hpp"
#include "geo_tuner/core/yaml_double.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;

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

std::string local_stamp()
{
  char buf[32];
  const std::time_t t = std::time(nullptr);
  std::tm tm_local{};
  localtime_r(&t, &tm_local);
  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_local);
  return buf;
}

double round_to(double v, int digits)
{
  const double f = std::pow(10.0, digits);
  return std::round(v * f) / f;
}

std::vector<std::string> split_axes(const std::string & spec)
{
  std::vector<std::string> out;
  std::stringstream ss(spec);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto b = item.find_first_not_of(" \t");
    if (b == std::string::npos) {continue;}
    const auto e = item.find_last_not_of(" \t");
    const std::string a = item.substr(b, e - b + 1);
    if (a == "x" || a == "y" || a == "z" || a == "yaw") {out.push_back(a);}
  }
  return out;
}

std::string join(const std::vector<std::string> & v, const char * sep)
{
  std::string s;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) {s += sep;}
    s += v[i];
  }
  return s;
}

/// Wait-free readiness test on a shared_future.
template<typename F>
bool ready(const F & f)
{
  return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

}  // namespace

const char * to_string(TunerState s)
{
  switch (s) {
    case TunerState::WAIT_ODOM: return "WAIT_ODOM";
    case TunerState::WAIT_ENABLE: return "WAIT_ENABLE";
    case TunerState::WAIT_OFFBOARD: return "WAIT_OFFBOARD";
    case TunerState::TOO_LOW: return "TOO_LOW";
    case TunerState::GOTO_HOVER: return "GOTO_HOVER";
    case TunerState::SETTLE: return "SETTLE";
    case TunerState::STEP: return "STEP";
    case TunerState::ANALYZE: return "ANALYZE";
    case TunerState::IDENTIFY: return "IDENTIFY";
    case TunerState::UPDATE_GAINS: return "UPDATE_GAINS";
    case TunerState::DONE: return "DONE";
    case TunerState::ABORT: return "ABORT";
  }
  return "UNKNOWN";
}

bool is_active_state(TunerState s)
{
  return s == TunerState::TOO_LOW || s == TunerState::GOTO_HOVER ||
         s == TunerState::SETTLE ||
         s == TunerState::STEP || s == TunerState::ANALYZE ||
         s == TunerState::IDENTIFY || s == TunerState::UPDATE_GAINS;
}

TuningConductor::TuningConductor(const rclcpp::NodeOptions & options)
: rclcpp::Node("tuning_conductor", options)
{
  // ---- parameters ----
  const auto controller_node =
    declare_parameter<std::string>("controller_node", "geometric_controller_node");
  const auto setpoint_topic = declare_parameter<std::string>(
    "setpoint_topic", "geometric_controller/multi_dof_setpoint");
  const auto odom_topic =
    declare_parameter<std::string>("odom_topic", "geometric_controller/odom");
  rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
  // Where the session hovers while it tunes.
  //
  // "capture" is the field default and the reason this node no longer
  // commands a climb: the hover point IS the point the pilot hands the
  // vehicle over at, captured the instant the session starts flying, so
  // the setpoint never jumps. "fixed" keeps the old behaviour for
  // simulators, which have no pilot to hand anything over -- and even
  // there the setpoint now ramps to the point instead of stepping to it.
  hover_mode_ = declare_parameter<std::string>("hover_mode", "capture");
  if (hover_mode_ != "capture" && hover_mode_ != "fixed") {
    RCLCPP_ERROR(
      get_logger(), "hover_mode must be \"capture\" or \"fixed\", got \"%s\"; using capture",
      hover_mode_.c_str());
    hover_mode_ = "capture";
  }
  hover_ = declare_parameter<std::vector<double>>("hover_position", {0.0, 0.0, 3.0});
  hover_yaw_param_ = declare_parameter<double>("hover_yaw", 0.0);          // rad, fixed mode
  hover_speed_ = declare_parameter<double>("hover_approach_speed", 0.7);   // m/s
  hover_yaw_rate_ = declare_parameter<double>("hover_approach_yaw_rate", 0.5);  // rad/s
  // Altitude gate. The session refuses to start below this height above
  // ground and holds position instead -- it never climbs to reach it.
  min_tuning_altitude_ = declare_parameter<double>("min_tuning_altitude", 5.0);  // m AGL
  // 1.0, not 0.5: the 2026-09-10 flight bottomed at 2.40 m AGL against a
  // 2.00 m floor -- down-steps overshot ~20 % and the settle drift ate the
  // rest of the old 0.5x margin. 1.0 makes the clearance 2x the step (T6).
  z_step_margin_ = declare_parameter<double>("z_step_margin", 1.0);
  // AGL source. Odometry z is NOT height above ground: PX4's local frame
  // origin sat 6.6 m below the ground in the first field session, which
  // put every altitude limit 6.6 m out. rel_alt is height above the home
  // point; ground_z is the fallback offset when no such topic exists.
  const auto agl_topic =
    declare_parameter<std::string>("agl_topic", "mavros/global_position/rel_alt");
  agl_timeout_ = declare_parameter<double>("agl_timeout", 2.0);            // s
  ground_z_ = declare_parameter<double>("ground_z", 0.0);                  // m, odom frame
  // A configured AGL topic is REQUIRED -- to start and for the whole session.
  // Deciding from whether a message happened to have arrived at hover capture
  // raced the topic's first message and silently put the floor on odometry z.
  agl_required_ = !agl_topic.empty();
  sched_.step_size = declare_parameter<double>("step_size", 0.5);        // m
  sched_.step_size_z = declare_parameter<double>("step_size_z", 0.4);    // m
  // 1.5 s, not 4: the acceleration-loop identification needs no parked
  // start, and the quiet gate (6 cm on all axes) is below 2026-09-14's 7 cm
  // hover wander, so 27 of 40 settles ran to the old 4 s cap -- 126 s of a
  // 313 s session. Quiet still ends it sooner; trim learning is not bound by it.
  settle_time_ = declare_parameter<double>("settle_time", 1.5);          // s before each step
  hover_timeout_ = declare_parameter<double>("hover_timeout", 20.0);     // s to reach hover point
  // 8 s, not 6: rung-2 rise times measured 2.0-3.6 s on 2026-09-10, and a
  // de-tuned rung must still show the fit >= 2 rise times (T3).
  episode_time_ = declare_parameter<double>("episode_time", 8.0);        // s of recording
  // Directory for raw per-episode CSVs (t,y per odometry sample). Empty
  // disables. The report records only the fits; these are the data behind
  // them, for offline analysis of sessions whose estimates disagree.
  episode_dump_dir_ = declare_parameter<std::string>("episode_dump_dir", "");
  // comma-separated to dodge YAML 1.1 parsing of bare "y" as a bool;
  // may include "yaw" for heading-loop identification
  sched_.axes = split_axes(declare_parameter<std::string>("axes", "z,x,y,yaw"));  // z first
  sched_.yaw_step = declare_parameter<double>("yaw_step", 0.5);          // rad
  // Hard ceiling on any live step-amplitude change. The real limit is
  // usually safety.max_pos_error (a step commands exactly that much
  // instantaneous position error, so a step at or above it aborts the
  // session on the spot) -- validation uses whichever is smaller. Raise
  // it deliberately, never by accident.
  sched_.max_step_size = declare_parameter<double>("max_step_size", 2.0);   // m
  sched_.min_step_size = declare_parameter<double>("min_step_size", 0.05);  // m
  sched_.max_yaw_step = declare_parameter<double>("max_yaw_step", 1.0);     // rad
  // Below this the response is small against odometry noise and the
  // fit-quality gate (nrmse < 0.15) starts rejecting episodes: the
  // session still refuses to act on bad fits, it just gets slower and may
  // end with "keeping gains". Warned about, not blocked.
  sched_.small_step_warn = declare_parameter<double>("small_step_warn", 0.25);  // m
  // How long to wait for the controller's parameter service to answer the
  // baseline gain read before asking again.
  service_timeout_ = declare_parameter<double>("service_timeout", 5.0);   // s
  yaw_T_target_ = declare_parameter<double>("yaw_time_constant", 0.35);   // s, target T
  yaw_tau_min_ = declare_parameter<double>("yawctrl_tau_min", 0.15);
  yaw_tau_max_ = declare_parameter<double>("yawctrl_tau_max", 1.2);
  // Per-episode quality screen for the yaw fit. The closed-loop (position)
  // fit keeps its 0.15: yaw is looser because a 0.5 rad heading response
  // carries more relative sensor noise, and the real gate on what reaches
  // yawctrl_tau is the bucket median + estimate_consistency. On 2026-09-09
  // the fixed 0.15 rejected 5 of 6 yaw episodes at nrmse 0.148-0.172 whose
  // T estimates agreed to ~1.3x -- consistent evidence thrown away.
  yaw_fit_nrmse_ = declare_parameter<double>("yaw_fit_nrmse", 0.25);
  // ---- design and decision (core/loop_design.hpp) ----
  // One bandwidth request, not a ladder: the lag-aware design lowers it on
  // its own when the identified lag would not leave the phase margin.
  decision_.wn_target = declare_parameter<double>("wn_target", 1.6);
  wn_target_z_ = declare_parameter<double>("wn_target_z", -1.0);
  if (wn_target_z_ <= 0.0) {wn_target_z_ = decision_.wn_target;}
  zeta_target_ = declare_parameter<double>("zeta_target", 0.95);
  decision_.zeta = zeta_target_;
  // Classical robustness pair, not fitted to any flight: nominal margin at
  // the identified plant, and a floor on the worst plant in the interval.
  decision_.margins.nominal_deg = declare_parameter<double>("pm_nominal_deg", 45.0);
  decision_.margins.worst_deg = declare_parameter<double>("pm_worst_deg", 35.0);
  // Materiality: gains within this fraction of the range the evidence
  // supports are left alone.
  decision_.gain_tolerance = declare_parameter<double>("gain_tolerance", 0.10);
  // Confirming gains needs an alpha interval at most this wide (hi/lo);
  // wider evidence flies another round instead.
  decision_.max_alpha_ci_ratio = declare_parameter<double>("max_alpha_ci_ratio", 1.19);
  max_change_ = declare_parameter<double>("max_gain_change_factor", 1.6);
  decision_.max_change = max_change_;
  // Rounds: identify -> apply -> validate. Three lets a rate-limited
  // update continue once and still be validated.
  max_rounds_ = std::max(1, static_cast<int>(declare_parameter<int>("max_rounds", 3)));
  // Degrees of phase margin estimation noise is allowed to cost: gains in
  // force stand, and validated gains pass, while within this of the spec.
  pm_tolerance_ = declare_parameter<double>("pm_tolerance_deg", 5.0);
  decision_.pm_hysteresis_deg = pm_tolerance_;
  max_overshoot_ = declare_parameter<double>("validation_max_overshoot", 0.25);
  yaw_tolerance_ = declare_parameter<double>("yaw_tolerance", 0.25);
  // Steps per axis per round. 4 = one full bidirectional cycle
  // (0 -> +d -> 0 -> -d -> 0), so a round never leaves the vehicle offset.
  sched_.episodes_per_rung =
    std::max(1, static_cast<int>(declare_parameter<int>("episodes_per_round", 4)));
  sched_.min_episodes = sched_.episodes_per_rung;
  // Yaw's per-episode time constants must agree this well to act on.
  consistency_ = declare_parameter<double>("estimate_consistency", 1.35);
  // Identification (core/accel_loop_id.hpp).
  id_cfg_.fs = rate_hz_;
  id_cfg_.confidence = declare_parameter<double>("id_confidence", 0.90);
  id_cfg_.min_excited_s = declare_parameter<double>("id_min_excited_s", 1.5);
  id_cfg_.min_r2 = declare_parameter<double>("id_min_r2", 0.6);
  // The node whose enable_thrust_estimator must read false before a session
  // starts: that estimator adapts the plant gain while it is measured. A bare
  // name takes controller_node's namespace. "" skips the check (only for
  // simulators that have no geometric_mavros node).
  estimator_node_ = declare_parameter<std::string>("estimator_node", "geometric_mavros_node");
  const auto cmd_topic =
    declare_parameter<std::string>("cmd_topic", "geometric_controller/cmd");

  // Parameters of the retired ladder/bucket logic. A profile still setting
  // them would silently not do what it says: say so.
  for (const char * gone : {"wn_ladder", "episodes_per_rung", "max_extra_episodes",
      "lag_mode", "stability_margin", "min_episodes_per_rung", "early_stop_spread",
      "yaw_final_rung_only"})
  {
    if (get_node_parameters_interface()->get_parameter_overrides().count(gone)) {
      RCLCPP_WARN(
        get_logger(), "parameter '%s' no longer exists and is ignored (see wn_target, "
        "episodes_per_round, max_rounds)", gone);
    }
  }
  // Bidirectional episodes: instead of flying back to the hover point and
  // throwing that leg away, record it. The setpoint walks
  // 0 -> +d -> 0 -> -d -> 0 ...; every leg is a clean step of size d from
  // a settled state, so the episode yield per unit time doubles and the
  // excursion envelope is unchanged.
  sched_.bidirectional = declare_parameter<bool>("bidirectional_episodes", true);
  // Adaptive settle: leave SETTLE as soon as the vehicle is quiet for
  // settle_quiet_time instead of always burning settle_time (which stays
  // the hard cap, so this is never slower).
  min_settle_time_ = declare_parameter<double>("min_settle_time", 0.3);
  sched_.settle_quiet_time = declare_parameter<double>("settle_quiet_time", 0.4);
  // Steady offset above which the SETTLE-phase trim learner acts (T4).
  // Deliberately below hover_capture_radius: the GOTO_HOVER learner only
  // ever ran when capture FAILED for 5 s, which on the 2026-09-10 flight
  // meant one update in 163 s while an unabsorbed [-0.24,-0.22,+0.34]
  // m/s^2 bias skewed every low-gain episode. SETTLE is where the vehicle
  // is parked and quiet by construction, so that is where the offset is
  // measured.
  trim_update_threshold_ = declare_parameter<double>("trim_update_threshold", 0.05);  // m
  sched_.settle_tol_pos = declare_parameter<double>("settle_tol_pos", 0.06);   // m
  sched_.settle_tol_vel = declare_parameter<double>("settle_tol_vel", 0.10);   // m/s
  sched_.settle_tol_yaw = declare_parameter<double>("settle_tol_yaw", 0.05);   // rad
  // Adaptive episode length: stop recording once the response has been
  // flat at its steady state for episode_quiet_time, but never before the
  // transient can have finished (min_episode_time and
  // episode_settle_periods/(zeta*wn)). episode_time is the cap.
  adaptive_episode_ = declare_parameter<bool>("adaptive_episode", true);
  episode_end_ = declare_parameter<std::string>("episode_end", "motion");
  if (episode_end_ != "motion" && episode_end_ != "settled") {
    RCLCPP_WARN(get_logger(), "episode_end '%s' unknown; using 'motion'", episode_end_.c_str());
    episode_end_ = "motion";
  }
  sched_.park_time = declare_parameter<double>("park_time", 1.0);        // s
  sched_.park_spread = declare_parameter<double>("park_spread", 0.03);   // m
  saturation_margin_ = declare_parameter<double>("saturation_margin", 0.8);
  min_episode_time_ = declare_parameter<double>("min_episode_time", 2.0);      // s
  sched_.episode_quiet_time = declare_parameter<double>("episode_quiet_time", 0.5);  // s
  sched_.episode_settle_band =
    declare_parameter<double>("episode_settle_band", 0.04);  // of |step|
  episode_settle_periods_ = declare_parameter<double>("episode_settle_periods", 4.0);
  // Both quiet gates are also read as a FRACTION of the step being flown,
  // and the tighter of the two applies. Without this a small
  // (confined-space) step would be declared settled while the residual is
  // still a large part of the step itself.
  sched_.settle_tol_frac = declare_parameter<double>("settle_tol_frac", 0.15);  // of |step|
  // ...and the band gets an absolute floor, so a small step does not ask
  // for a flatness finer than the odometry noise (which would silently
  // disable the adaptive stop). Axis units: m, or rad.
  sched_.episode_settle_floor = declare_parameter<double>("episode_settle_floor", 0.01);
  // Capture radius for "arrived at the hover point" before settling.
  hover_capture_radius_ = declare_parameter<double>("hover_capture_radius", 0.3);  // m
  hover_capture_speed_ = declare_parameter<double>("hover_capture_speed", 0.3);    // m/s
  // Hold at WAIT_ENABLE until the ~/start service is called, rather than
  // starting as soon as the vehicle is in OFFBOARD.
  require_enable_ = declare_parameter<bool>("require_enable", false);
  // OFFBOARD supervision: episodes only run while PX4 is in OFFBOARD
  // (otherwise the vehicle ignores our setpoints and every fit is
  // garbage). Disable only for plant simulators without mavros.
  require_offboard_ = declare_parameter<bool>("require_offboard", true);
  const auto mavros_state_topic =
    declare_parameter<std::string>("mavros_state_topic", "mavros/state");
  offboard_mode_ = declare_parameter<std::string>("offboard_mode", "OFFBOARD");
  report_path_ =
    declare_parameter<std::string>("report_path", "/tmp/geo_tuner_report.yaml");
  // One knob for a field day: when set, every session writes a timestamped
  // report AND its raw episode CSVs under this directory, so consecutive
  // sessions never overwrite each other. Overrides report_path and
  // episode_dump_dir. Exposed as a launch argument on field_tune.launch.py.
  output_dir_ = declare_parameter<std::string>("output_dir", "");
  // safety limits
  SafetyLimits limits;
  limits.max_tilt = declare_parameter<double>("safety.max_tilt", 0.6);
  limits.max_pos_error = declare_parameter<double>("safety.max_pos_error", 2.0);
  limits.max_velocity = declare_parameter<double>("safety.max_velocity", 4.0);
  limits.min_altitude = declare_parameter<double>("safety.min_altitude", 1.0);
  limits.max_altitude = declare_parameter<double>("safety.max_altitude", 40.0);
  limits.odom_timeout = declare_parameter<double>("safety.odom_timeout", 0.4);
  limits.osc_rate_rms = declare_parameter<double>("safety.osc_rate_rms", 1.2);
  safety_ = SafetyMonitor(limits);

  // Degenerate configuration must not reach the state machine: every state
  // indexes axes[axis_idx] unguarded, and a vehicle in
  // OFFBOARD is a bad place to find out the list was empty.
  if (hover_.size() != 3) {
    RCLCPP_ERROR(
      get_logger(), "hover_position needs 3 elements, got %zu; using [0, 0, 3]",
      hover_.size());
    hover_ = {0.0, 0.0, 3.0};
  }
  if (sched_.axes.empty()) {
    RCLCPP_ERROR(get_logger(), "axes is empty (want a subset of x,y,z,yaw); using z,x,y");
    sched_.axes = {"z", "x", "y"};
  }

  // ---- interfaces ----
  sp_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    setpoint_topic, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("geo_tuner/status", 10);
  // Structured status for a ground station. The String above is a human
  // log line that only appears when something happens; a panel needs the
  // state, the progress through the rounds, and the reason it is waiting,
  // at a steady rate.
  health_pub_ = create_publisher<DiagnosticStatus>("geo_tuner/health", 10);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::SensorDataQoS(),
    std::bind(&TuningConductor::odom_cb, this, std::placeholders::_1));
  if (!agl_topic.empty()) {
    agl_sub_ = create_subscription<std_msgs::msg::Float64>(
      agl_topic, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        agl_msg_ = msg->data;
        agl_msg_t_ = now_s();
      });
  }
#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
  if (require_offboard_) {
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      mavros_state_topic, 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {px4_mode_ = msg->mode;});
  }
#else
  (void)mavros_state_topic;
  if (require_offboard_) {
    RCLCPP_ERROR(
      get_logger(),
      "require_offboard is set but this build has no mavros_msgs: OFFBOARD "
      "supervision is unavailable. Rebuild with mavros_msgs, or set "
      "require_offboard:=false deliberately (simulators only).");
    require_offboard_ = false;
  }
#endif
  std::string ctrl = controller_node;
  while (!ctrl.empty() && ctrl.front() == '/') {ctrl.erase(ctrl.begin());}
  while (!ctrl.empty() && ctrl.back() == '/') {ctrl.pop_back();}
  get_param_cli_ = create_client<GetParameters>("/" + ctrl + "/get_parameters");
  cmd_sub_ = create_subscription<mav_controllers_ros::msg::SE3Command>(
    cmd_topic, 10, std::bind(&TuningConductor::cmd_cb, this, std::placeholders::_1));
  if (!estimator_node_.empty()) {
    std::string est = estimator_node_;
    while (!est.empty() && est.front() == '/') {est.erase(est.begin());}
    if (est.find('/') == std::string::npos) {
      const auto slash = ctrl.rfind('/');
      if (slash != std::string::npos) {est = ctrl.substr(0, slash) + "/" + est;}
    }
    estimator_node_ = est;
    estimator_param_cli_ = create_client<GetParameters>("/" + est + "/get_parameters");
  }
  set_param_cli_ = create_client<SetParameters>("/" + ctrl + "/set_parameters");

  // ---- state ----
  state_t0_ = now_s();
  std::copy_n(hover_.begin(), 3, hover_pt_.begin());
  hover_yaw_ = hover_yaw_param_;
  sched_.setpoint = hover_pt_;
  sched_.setpoint_yaw = hover_yaw_;

  // Session control from the ground. Without these a session could only be
  // started by launching the node and stopped by killing it -- over a
  // field datalink, in flight.
  auto trig = [](auto fn, TuningConductor * self) {
      return [self, fn](
        const std::shared_ptr<rmw_request_id_t>, Trigger::Request::SharedPtr req,
        Trigger::Response::SharedPtr res) {(self->*fn)(req, res);};
    };
  start_srv_ = create_service<Trigger>("~/start", trig(&TuningConductor::srv_start, this));
  abort_srv_ = create_service<Trigger>("~/abort", trig(&TuningConductor::srv_abort, this));
  accept_srv_ =
    create_service<Trigger>("~/accept", trig(&TuningConductor::srv_accept, this));
  restore_srv_ = create_service<Trigger>(
    "~/restore_safe", trig(&TuningConductor::srv_restore_safe, this));
  reset_srv_ = create_service<Trigger>("~/reset", trig(&TuningConductor::srv_reset, this));

  start_requested_ = !require_enable_;
  // Step amplitudes are live-settable (RViz tuner panel / ros2 param set)
  // so the envelope can be trimmed to the available space without
  // restarting a session.
  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&TuningConductor::on_set_parameters, this, std::placeholders::_1));
  const std::vector<std::tuple<std::string, double, bool>> to_check{
    {"step_size", sched_.step_size, false},
    {"step_size_z", sched_.step_size_z, false},
    {"yaw_step", sched_.yaw_step, true}};
  for (const auto & [name, val, is_yaw] : to_check) {
    const auto [ok, why] =
      sched_.validate_step(val, is_yaw, safety_.limits().max_pos_error);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "%s=%s: %s", name.c_str(), fmt(val, 6).c_str(), why.c_str());
    } else if (!why.empty()) {
      RCLCPP_WARN(get_logger(), "%s=%s: %s", name.c_str(), fmt(val, 6).c_str(), why.c_str());
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "Step envelope: +/-%s m lateral, +/-%s m vertical, +/-%s rad yaw about the "
    "hover point (max_pos_error %s m)",
    fmt(sched_.step_size, 2).c_str(), fmt(sched_.step_size_z, 2).c_str(),
    fmt(sched_.yaw_step, 2).c_str(), fmt(safety_.limits().max_pos_error, 2).c_str());
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / rate_hz_),
    std::bind(&TuningConductor::tick, this));
  health_timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&TuningConductor::publish_health, this));

  std::ostringstream hov;
  if (hover_mode_ == "fixed") {
    hov << "fixed [" << fmt(hover_[0], 2) << ", " << fmt(hover_[1], 2) << ", "
        << fmt(hover_[2], 2) << "] (ramped at " << fmt(hover_speed_, 2) << " m/s)";
  } else {
    hov << "captured where " << offboard_mode_ << " is engaged";
  }
  RCLCPP_INFO(
    get_logger(),
    "Tuning conductor up. axes=[%s] wn_target=%s (z %s) zeta=%s PM>=%s/%s deg "
    "rounds<=%d, %d steps/axis/round, hover=%s, estimator check: %s",
    join(sched_.axes, ", ").c_str(), fmt(decision_.wn_target, 2).c_str(),
    fmt(wn_target_z_, 2).c_str(), fmt(zeta_target_, 2).c_str(),
    fmt(decision_.margins.nominal_deg, 0).c_str(), fmt(decision_.margins.worst_deg, 0).c_str(),
    max_rounds_, sched_.episodes_per_rung, hov.str().c_str(),
    estimator_node_.empty() ? "OFF (estimator_node empty)" : ("/" + estimator_node_).c_str());
  RCLCPP_INFO(
    get_logger(),
    "Altitude gate: start at >= %s m AGL (min_tuning_altitude %s, safety floor %s "
    "+ %s for a %s m down step)",
    fmt(min_start_altitude(), 2).c_str(), fmt(min_tuning_altitude_, 2).c_str(),
    fmt(safety_.limits().min_altitude, 2).c_str(), fmt(z_step_clearance(), 2).c_str(),
    fmt(sched_.step_size_z, 2).c_str());
}

// ---------------------------------------------------------------------
// live step-amplitude configuration

rcl_interfaces::msg::SetParametersResult TuningConductor::on_set_parameters(
  const std::vector<rclcpp::Parameter> & params)
{
  // Live reconfiguration of the step amplitudes.
  //
  // Everything else stays launch-time: these three are the knobs a pilot
  // needs to trim the manoeuvre envelope to the space actually available,
  // and they are the ones the RViz tuner panel exposes. A change takes
  // effect at the NEXT episode -- never mid-step -- and the schedule
  // self-corrects, because a leg's step is measured as the difference
  // between the commanded offsets, not assumed.
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  const std::map<std::string, bool> live{
    {"step_size", false}, {"step_size_z", false}, {"yaw_step", true}};

  // The working altitude is the other live knob: the panel sets it, and it
  // is refused rather than silently clipped when it leaves no room for a
  // downward z step above the safety floor.
  for (const auto & p : params) {
    if (p.get_name() != "min_tuning_altitude") {continue;}
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      result.successful = false;
      result.reason = "min_tuning_altitude must be a number";
      return result;
    }
    const double value = p.as_double();
    const double need = safety_.limits().min_altitude + z_step_clearance();
    if (value < need) {
      result.successful = false;
      result.reason =
        "min_tuning_altitude " + fmt(value, 2) + " m leaves no room for a " +
        fmt(sched_.step_size_z, 2) + " m down step above the " +
        fmt(safety_.limits().min_altitude, 2) + " m safety floor; need >= " +
        fmt(need, 2) + " m";
      RCLCPP_WARN(get_logger(), "rejected %s", result.reason.c_str());
      return result;
    }
    if (value > safety_.limits().max_altitude) {
      result.successful = false;
      result.reason =
        "min_tuning_altitude " + fmt(value, 2) + " m is above safety.max_altitude " +
        fmt(safety_.limits().max_altitude, 2) + " m";
      return result;
    }
  }

  for (const auto & p : params) {
    const auto it = live.find(p.get_name());
    if (it == live.end()) {continue;}
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      result.successful = false;
      result.reason = p.get_name() + " must be a number";
      return result;
    }
    const double value = p.as_double();
    // A bigger vertical step lowers the bottom of the manoeuvre. Refuse
    // one that would fly the current hover point into the floor rather
    // than discovering it one down-leg later.
    if (p.get_name() == "step_size_z" && is_active_state(state_)) {
      const auto agl = hover_agl();
      const double bottom = value * (1.0 + std::max(0.0, z_step_margin_));
      if (agl && *agl - bottom < safety_.limits().min_altitude) {
        result.successful = false;
        result.reason =
          "step_size_z " + fmt(value, 2) + " m would take the vehicle to " +
          fmt(*agl - bottom, 2) + " m AGL, below the " +
          fmt(safety_.limits().min_altitude, 2) + " m floor; climb or use a smaller step";
        RCLCPP_WARN(get_logger(), "rejected %s", result.reason.c_str());
        return result;
      }
    }
    if (p.get_name() != "yaw_step") {
      const auto why = step_saturation_reason(value, p.get_name() == "step_size_z");
      if (!why.empty()) {
        result.successful = false;
        result.reason = p.get_name() + " " + fmt(value, 2) + " m: " + why;
        RCLCPP_WARN(get_logger(), "rejected %s", result.reason.c_str());
        return result;
      }
    }
    const auto [ok, note] =
      sched_.validate_step(value, it->second, safety_.limits().max_pos_error);
    if (!ok) {
      RCLCPP_WARN(
        get_logger(), "rejected %s=%s: %s", p.get_name().c_str(),
        fmt(value, 6).c_str(), note.c_str());
      result.successful = false;
      result.reason = p.get_name() + ": " + note;
      return result;
    }
    if (!note.empty()) {
      RCLCPP_WARN(
        get_logger(), "%s=%s: %s", p.get_name().c_str(), fmt(value, 6).c_str(),
        note.c_str());
    }
  }
  for (const auto & p : params) {
    const std::string & n = p.get_name();
    if (n == "step_size") {
      sched_.step_size = p.as_double();
    } else if (n == "step_size_z") {
      sched_.step_size_z = p.as_double();
    } else if (n == "yaw_step") {
      sched_.yaw_step = p.as_double();
    } else if (n == "min_tuning_altitude") {
      min_tuning_altitude_ = p.as_double();
      status(
        "min_tuning_altitude -> " + fmt(min_tuning_altitude_, 2) +
        " m AGL (start gate " + fmt(min_start_altitude(), 2) + " m)");
      continue;
    } else {
      continue;
    }
    status(n + " -> " + fmt(p.as_double(), 2) + " (applies from the next episode)");
  }
  return result;
}

// ---------------------------------------------------------------------

double TuningConductor::now_s()
{
  return static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
}

bool TuningConductor::in_offboard() const
{
  return !require_offboard_ || (px4_mode_ && *px4_mode_ == offboard_mode_);
}

void TuningConductor::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  OdomSample s;
  s.t = now_s();
  // The episode time axis must live in the sensor's clock, not this node's:
  // under a simulator running slower than real time (or a delayed odometry
  // pipeline) wall-clock arrival times stretch the apparent response and
  // bias the identified dynamics. Staleness detection stays on the receiver
  // clock (s.t) — the two clocks may hold a constant offset that would look
  // like staleness. Fall back to arrival time for unstamped publishers.
  const double stamp = static_cast<double>(msg->header.stamp.sec) +
    1e-9 * static_cast<double>(msg->header.stamp.nanosec);
  s.t_stamp = stamp > 0.0 ? stamp : s.t;
  s.pos = {msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z};
  s.vel = {msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  s.quat = {msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z};
  s.body_rates = {msg->twist.twist.angular.x, msg->twist.twist.angular.y,
    msg->twist.twist.angular.z};
  // Altitude limits are judged on height above ground, not odometry z.
  agl_fresh_ = agl_msg_ && (now_s() - agl_msg_t_) < agl_timeout_;
  s.agl = agl_fresh_ ? *agl_msg_ : s.pos[2] - ground_z_;
  sched_.odom = s;
}

std::optional<double> TuningConductor::agl_now() const
{
  if (!sched_.odom) {return std::nullopt;}
  return sched_.odom->agl;
}

const char * TuningConductor::agl_source() const
{
  return agl_fresh_ ? "agl_topic" : "odom_z-ground_z";
}

std::optional<double> TuningConductor::hover_agl() const
{
  const auto agl = agl_now();
  if (!agl || !sched_.odom) {return std::nullopt;}
  // The hover point is expressed in the odometry frame; shift the measured
  // AGL by how far above the vehicle that point sits.
  return *agl + (hover_pt_[2] - sched_.odom->pos[2]);
}

double TuningConductor::z_step_clearance() const
{
  // A downward z leg commands the setpoint step_size_z below the hover
  // point, and the response undershoots that by a fraction of the step
  // before it settles. Both scale with the step amplitude, which is a live
  // parameter -- so this moves whenever the operator changes it.
  return sched_.step_size_z * (1.0 + std::max(0.0, z_step_margin_));
}

double TuningConductor::min_start_altitude() const
{
  // Two independent requirements, whichever binds: the operator's chosen
  // working altitude, and the room a downward z step needs above the
  // safety floor. Tuning at the floor with a 0.4 m step means flying into
  // it.
  return std::max(
    min_tuning_altitude_, safety_.limits().min_altitude + z_step_clearance());
}

std::pair<bool, std::string> TuningConductor::altitude_gate() const
{
  if (agl_required_ && !agl_fresh_) {
    return {false, "no height-above-ground message in the last " + fmt(agl_timeout_, 1) +
      " s: not starting with altitude limits on odometry z"};
  }
  const auto agl = hover_agl();
  if (!agl) {return {false, "no altitude yet"};}
  const double need = min_start_altitude();
  if (*agl >= need) {return {true, ""};}
  const double floor_need = safety_.limits().min_altitude + z_step_clearance();
  std::string why = "too low to tune: " + fmt(*agl, 1) + " m AGL, need " +
    fmt(need, 1) + " m";
  why += need > min_tuning_altitude_ ?
    " (safety floor " + fmt(safety_.limits().min_altitude, 1) + " m + " +
    fmt(z_step_clearance(), 1) + " m for a " + fmt(sched_.step_size_z, 2) +
    " m down step)" :
    " (min_tuning_altitude; the z step needs " + fmt(floor_need, 1) + " m)";
  return {false, why};
}

void TuningConductor::capture_hover()
{
  if (hover_mode_ == "fixed") {
    std::copy_n(hover_.begin(), 3, hover_pt_.begin());
    hover_yaw_ = hover_yaw_param_;
  } else if (sched_.odom) {
    hover_pt_ = sched_.odom->pos;
    hover_yaw_ = TuningSchedule::yaw_of(sched_.odom->quat);
  }
  hover_captured_ = true;
  sched_.leg_offset = 0.0;
}

void TuningConductor::hold_here()
{
  if (!sched_.odom) {return;}
  sched_.setpoint = sched_.odom->pos;
  sched_.setpoint_yaw = TuningSchedule::yaw_of(sched_.odom->quat);
  sched_.leg_offset = 0.0;
}

void TuningConductor::begin_flying()
{
  capture_hover();
  const auto [ok, why] = altitude_gate();
  if (!ok) {
    // Refuse, and hold exactly where the vehicle is. The vehicle is in
    // OFFBOARD under our setpoints now, so "refuse" cannot mean "stop
    // publishing"; it means "command the point it is already at".
    hold_here();
    too_low_log_t_ = now_s();
    status(
      why + ". Take manual control, climb, then re-engage " + offboard_mode_ +
      " -- or lower min_tuning_altitude from the ground station.");
    goto_state(TunerState::TOO_LOW);
    return;
  }
  // Command-limit gate, same shape: hold, say why, and start by itself once
  // the operator lowers the step from the panel (st_too_low re-checks).
  if (const auto sat = saturation_gate(); !sat.empty()) {
    hold_here();
    too_low_log_t_ = now_s();
    status(sat + ". Lower it from the ground station; the session starts once it fits.");
    goto_state(TunerState::TOO_LOW);
    return;
  }
  if (session_t0_ < 0.0) {
    session_t0_ = now_s();
    // The startup lines print the configured envelope; the panel can change
    // it before the start (2026-09-14 flew 1 m steps under a logged 0.5/0.4 m
    // envelope and a 2.80 m floor clearance). Record what this session flies.
    const auto agl = hover_agl();
    status(
      "Session envelope: steps " + fmt(sched_.step_size, 2) + " m lateral, " +
      fmt(sched_.step_size_z, 2) + " m vertical, " + fmt(sched_.yaw_step, 2) +
      " rad yaw; start gate " + fmt(min_start_altitude(), 2) + " m AGL (hover at " +
      (agl ? fmt(*agl, 2) : std::string("?")) + " m)");
  }
  // The schedule may be parked on an axis already finished (a resume after
  // an OFFBOARD pause): move to one this round still flies.
  if (!seek_active_axis()) {return;}
  // In capture mode the setpoint is already the hover point; in fixed mode
  // GOTO_HOVER ramps to it.
  goto_state(TunerState::GOTO_HOVER);
}

void TuningConductor::publish_setpoint()
{
  trajectory_msgs::msg::MultiDOFJointTrajectory msg;
  msg.header.stamp = get_clock()->now();
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint pt;
  geometry_msgs::msg::Transform tr;
  tr.translation.x = sched_.setpoint[0];
  tr.translation.y = sched_.setpoint[1];
  tr.translation.z = sched_.setpoint[2];
  tr.rotation.w = std::cos(0.5 * sched_.setpoint_yaw);
  tr.rotation.z = std::sin(0.5 * sched_.setpoint_yaw);
  pt.transforms.push_back(tr);
  pt.velocities.emplace_back();
  geometry_msgs::msg::Twist acc;
  acc.linear.x = a_trim_[0];
  acc.linear.y = a_trim_[1];
  acc.linear.z = a_trim_[2];
  pt.accelerations.push_back(acc);
  msg.points.push_back(pt);
  sp_pub_->publish(msg);
}

void TuningConductor::status(const std::string & text, bool log)
{
  if (log) {
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }
  std_msgs::msg::String m;
  m.data = std::string("[") + to_string(state_) + "] " + text;
  status_pub_->publish(m);
}

void TuningConductor::goto_state(TunerState s)
{
  state_ = s;
  state_t0_ = now_s();
}

// ---------------------------------------------------------------------
// gain get/set through the controller's parameter interface

void TuningConductor::request_gains(std::optional<double> now)
{
  auto req = std::make_shared<GetParameters::Request>();
  req->names = {"gains.pos.x", "gains.pos.y", "gains.pos.z",
    "gains.vel.x", "gains.vel.y", "gains.vel.z",
    "yawctrl_tau", "attctrl_tau", "mass", "max_accel", "max_tilt_angle"};
  auto fut = get_param_cli_->async_send_request(req);
  get_req_id_ = fut.request_id;
  pending_get_ = fut.future.share();
  request_t0_ = now ? *now : now_s();
  ++request_tries_;
}

void TuningConductor::apply_yaw_tau(double tau)
{
  apply_gains({}, tau);
}

void TuningConductor::apply_gains(const Gains & gains, std::optional<double> yaw_tau)
{
  // One request for everything a round changes: two in-flight requests
  // would share the single pending_set_ slot and one answer would be lost.
  auto req = std::make_shared<SetParameters::Request>();
  if (yaw_tau) {
    rcl_interfaces::msg::Parameter p;
    p.name = "yawctrl_tau";
    p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    p.value.double_value = *yaw_tau;
    req->parameters.push_back(p);
  }
  for (const auto & [ax, kxkv] : gains) {
    for (const auto & [name, val] :
      {std::pair<std::string, double>{"gains.pos." + ax, kxkv.first},
        std::pair<std::string, double>{"gains.vel." + ax, kxkv.second}})
    {
      rcl_interfaces::msg::Parameter p;
      p.name = name;
      p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
      p.value.double_value = val;
      req->parameters.push_back(p);
    }
  }
  pending_set_ = set_param_cli_->async_send_request(req).future.share();
}

// ---------------------------------------------------------------------

void TuningConductor::tick()
{
  const double now = now_s();

  // Safety runs while the conductor is actively flying the vehicle. It
  // deliberately does NOT run in WAIT_OFFBOARD: there the pilot / another
  // mode is in command and e.g. tilt limits don't apply.
  if (sched_.odom && is_active_state(state_)) {
    // TOO_LOW is the one state that is knowingly under the floor: it is
    // holding position because it refused to start there. Every other
    // limit still applies, and the floor applies again the moment it
    // starts flying episodes.
    auto violations = safety_.check(
      *sched_.odom, sched_.setpoint, /*check_min_altitude=*/state_ != TunerState::TOO_LOW);
    const auto stale = safety_.check_stale(now);
    violations.insert(violations.end(), stale.begin(), stale.end());
    if (!violations.empty()) {
      std::vector<std::string> texts;
      for (auto v : violations) {texts.emplace_back(to_string(v));}
      abort(join(texts, ", "));
    } else if (agl_required_ && !agl_fresh_ && state_ != TunerState::TOO_LOW) {
      // (TOO_LOW is where a session waits for the AGL topic: see altitude_gate.)
      // The floor was set on the AGL topic; odometry z has an unrelated
      // origin (6.6 m off on this vehicle). Hold rather than re-base it.
      abort("height-above-ground topic lost mid-session (no message for " +
        fmt(agl_timeout_, 1) + " s); not re-basing the altitude floor on odometry z");
    }
  }

  // After a session (finished or stopped) the conductor keeps publishing so
  // OFFBOARD stays holdable -- but while the pilot flies another mode, the
  // held point must follow the vehicle. Otherwise re-engaging OFFBOARD
  // after flying away snaps the vehicle back to where the session ended
  // (the 2026-09-08 runaway class). Trim was learned for the old point and
  // the old session: drop it.
  if ((state_ == TunerState::ABORT || state_ == TunerState::DONE) && !in_offboard()) {
    hold_here();
    a_trim_ = {0.0, 0.0, 0.0};
  }

  // Pilot/mode supervision: leaving OFFBOARD mid-session pauses the tuner
  // (episode discarded, gains kept) until OFFBOARD returns.
  if (is_active_state(state_) && !in_offboard()) {
    status(
      "PX4 left " + offboard_mode_ + " (now: " + px4_mode_.value_or("None") +
      "); pausing tuning");
    sched_.recording.clear();
    sched_.leg_offset = 0.0;
    sched_.setpoint_yaw = 0.0;
    sched_.reset_quiet();
    safety_.reset();
    goto_state(TunerState::WAIT_OFFBOARD);
  }

  // Setpoint stream must never stop while OFFBOARD is active (and it must
  // already flow in WAIT_OFFBOARD, or PX4 refuses the switch).
  if (state_ != TunerState::WAIT_ODOM && state_ != TunerState::WAIT_ENABLE) {
    publish_setpoint();
  }
  // Identification input: the setpoint just published, the latest odometry
  // and command, the gains in force.
  record_sample();

  switch (state_) {
    case TunerState::WAIT_ODOM: st_wait_odom(now); break;
    case TunerState::WAIT_ENABLE: st_wait_enable(now); break;
    case TunerState::WAIT_OFFBOARD: st_wait_offboard(now); break;
    case TunerState::TOO_LOW: st_too_low(now); break;
    case TunerState::GOTO_HOVER: st_goto_hover(now); break;
    case TunerState::SETTLE: st_settle(now); break;
    case TunerState::STEP: st_step(now); break;
    case TunerState::ANALYZE: st_analyze(now); break;
    case TunerState::IDENTIFY: st_identify(now); break;
    case TunerState::UPDATE_GAINS: st_update_gains(now); break;
    case TunerState::DONE: break;    // keep publishing hover setpoint
    case TunerState::ABORT: st_abort(now); break;   // hold; confirm the restore
  }
}

// ---- states ----

void TuningConductor::st_wait_odom(double now)
{
  if (!sched_.odom) {return;}
  // The controller's parameter service must actually be discovered before
  // we call it: an async call on an undiscovered service is simply
  // dropped, and the session would then sit in WAIT_ENABLE forever with
  // no explanation.
  if (!get_param_cli_->service_is_ready()) {
    if (!waiting_logged_) {
      status(
        std::string("Odometry received; waiting for the controller's parameter "
        "service (") + get_param_cli_->get_service_name() + ")");
      waiting_logged_ = true;
    }
    return;
  }
  status("Odometry received; requesting current gains");
  request_gains(now);
  goto_state(TunerState::WAIT_ENABLE);
}

void TuningConductor::st_wait_enable(double now)
{
  // Baseline already captured on an earlier tick: we are simply holding
  // for the operator to press start.
  if (baseline_read_) {
    if (start_requested_) {
      if (!estimator_ok(now)) {return;}
      if (in_offboard()) {
        begin_flying();
      } else {
        status("Start requested; waiting for PX4 mode " + offboard_mode_);
        goto_state(TunerState::WAIT_OFFBOARD);
      }
    }
    return;
  }

  // A dropped or unanswered request must not strand the session: ask
  // again (and say so) rather than waiting silently forever.
  if (pending_get_ && !ready(*pending_get_) && now - request_t0_ > service_timeout_) {
    get_param_cli_->remove_pending_request(get_req_id_);
    pending_get_.reset();
    status(
      "controller did not answer the gain read in " + fmt(service_timeout_, 0) +
      "s (attempt " + std::to_string(request_tries_) + "); retrying");
    return;
  }
  if (!pending_get_) {
    if (get_param_cli_->service_is_ready()) {
      request_gains(now);
    }
    return;
  }

  // Capture the controller's current gains as the known-safe baseline
  if (ready(*pending_get_)) {
    const auto res = pending_get_->get();
    pending_get_.reset();
    if (!res || res->values.size() < 6) {
      abort("could not read controller gains");
      return;
    }
    // Every later limit is relative to these, so they must be real numbers:
    // an integer-typed or unset parameter reads as double_value 0.0.
    std::vector<double> vals;
    for (size_t i = 0; i < 6; ++i) {
      using rcl_interfaces::msg::ParameterType;
      const auto & pv = res->values[i];
      const double val = pv.type == ParameterType::PARAMETER_DOUBLE ? pv.double_value :
        pv.type == ParameterType::PARAMETER_INTEGER ? static_cast<double>(pv.integer_value) :
        std::numeric_limits<double>::quiet_NaN();
      if (!std::isfinite(val) || val <= 0.0) {
        abort(std::string("controller gain ") + (i < 3 ? "gains.pos." : "gains.vel.") +
          "xyz"[i % 3] + " is not a positive number (type " + std::to_string(pv.type) +
          "): refusing to tune from it");
        return;
      }
      vals.push_back(val);
    }
    gains_ = Gains{{"x", {vals[0], vals[3]}},
      {"y", {vals[1], vals[4]}},
      {"z", {vals[2], vals[5]}}};
    safe_gains_ = gains_;
    baseline_gains_ = gains_;
    session_result_.clear();
    // A new session gets its own timestamp; with output_dir set that gives
    // it its own report file and episode folder, so a field day of repeated
    // sessions leaves one artifact per session instead of the last one.
    {
      const std::time_t t = std::time(nullptr);
      char buf[32];
      std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", std::localtime(&t));
      session_stamp_ = buf;
      episode_seq_ = 0;
      if (!output_dir_.empty()) {
        report_path_ =
          (std::filesystem::path(output_dir_) /
          ("geo_tuner_report_" + session_stamp_ + ".yaml")).string();
      }
    }
    for (const auto & [ax, kxkv] : *gains_) {
      const auto [wn, zeta] = wn_zeta_from_pd(kxkv.first, kxkv.second);
      status(
        "baseline " + ax + ": kx=" + fmt(kxkv.first, 2) + " kv=" +
        fmt(kxkv.second, 2) + " (wn=" + fmt(wn, 2) + ", zeta=" + fmt(zeta, 2) + ")");
    }
    // Yaw time constant: yawctrl_tau if the controller has it and it is
    // > 0, else it follows attctrl_tau. A controller without the
    // parameter (upstream build) can't be yaw-tuned.
    double yaw_tau = 0.0, att_tau = 0.0;
    if (res->values.size() >= 8) {
      using rcl_interfaces::msg::ParameterType;
      if (res->values[6].type == ParameterType::PARAMETER_DOUBLE) {
        yaw_tau = res->values[6].double_value;
      }
      if (res->values[7].type == ParameterType::PARAMETER_DOUBLE) {
        att_tau = res->values[7].double_value;
      }
    }
    if (att_tau > 0.0) {
      attctrl_tau_ = att_tau;
    }
    // The controller publishes mass * a_des; its own mass turns that back
    // into the commanded acceleration the identification needs.
    if (res->values.size() >= 9 &&
      res->values[8].type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE &&
      res->values[8].double_value > 0.0)
    {
      controller_mass_ = res->values[8].double_value;
    } else {
      abort(
        "controller has no positive 'mass' parameter: cannot convert its force command to "
        "the commanded acceleration identification needs");
      return;
    }
    // Command limits, for the saturation guard (unknown on controllers that
    // do not expose them: the guard then stays silent).
    {
      using rcl_interfaces::msg::ParameterType;
      const auto num = [&](size_t i) {
          if (res->values.size() <= i) {return 0.0;}
          const auto & pv = res->values[i];
          return pv.type == ParameterType::PARAMETER_DOUBLE ? pv.double_value :
                 pv.type == ParameterType::PARAMETER_INTEGER ?
                 static_cast<double>(pv.integer_value) : 0.0;
        };
      ctrl_max_accel_ = std::max(0.0, num(9));
      ctrl_max_tilt_ = std::max(0.0, num(10));
    }
    if (yaw_tau > 0.0) {
      yaw_tau_ = yaw_tau;
    } else if (att_tau > 0.0) {
      yaw_tau_ = att_tau;
    }
    baseline_yaw_tau_ = yaw_tau_;
    safe_yaw_tau_ = yaw_tau_;
    const bool wants_yaw =
      std::find(sched_.axes.begin(), sched_.axes.end(), "yaw") != sched_.axes.end();
    if (wants_yaw) {
      if (!yaw_tau_) {
        status(
          "controller has no yawctrl_tau/attctrl_tau params; skipping yaw axis");
        sched_.axes.erase(
          std::remove(sched_.axes.begin(), sched_.axes.end(), "yaw"),
          sched_.axes.end());
      } else {
        status(
          "baseline yaw: tau=" + fmt(*yaw_tau_, 3) + " (target T=" +
          fmt(yaw_T_target_, 2) + "s)");
      }
    }
    axis_prog_.clear();
    for (const auto & ax : sched_.axes) {axis_prog_[ax] = AxisProgress{};}
    baseline_read_ = true;
    if (require_enable_ && !start_requested_) {
      status("Gains read. Waiting for the ~/start service (require_enable is set)");
      return;
    }
    if (!estimator_ok(now)) {return;}
    if (!in_offboard()) {
      status(
        "Waiting for PX4 mode " + offboard_mode_ +
        " (setpoint stream active; switch modes to start)");
      goto_state(TunerState::WAIT_OFFBOARD);
    } else {
      begin_flying();
    }
  }
}

bool TuningConductor::estimator_ok(double now)
{
  // Precondition, checked on the LIVE node rather than trusted to launch
  // plumbing: on every tuning flight 2026-09-09..13 the launch-file pin that
  // was meant to turn this estimator off lost to the persisted override
  // (an exact node-name key beats a /** wildcard in ROS 2 parameter files),
  // and nothing noticed.
  if (estimator_node_.empty() || (estimator_off_ && *estimator_off_)) {return true;}
  if (pending_est_ && ready(*pending_est_)) {
    const auto res = pending_est_->get();
    pending_est_.reset();
    // Only an explicit boolean false passes: an undeclared or mistyped
    // parameter proves nothing about what the node is running.
    const bool readable = res && !res->values.empty() &&
      res->values[0].type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    const bool on = !readable || res->values[0].bool_value;
    estimator_off_ = !on;
    if (on) {
      // The node reads the flag once at startup: "ros2 param set" changes the
      // parameter without changing the estimator, so only a relaunch fixes it.
      const std::string why = readable ?
        "/" + estimator_node_ + " has enable_thrust_estimator=true: it adapts the plant "
        "gain this session measures. Relaunch the stack with the tuner session overrides "
        "(the node reads the flag only at startup; ros2 param set does NOT turn it off)" :
        "/" + estimator_node_ + " did not return a boolean enable_thrust_estimator: cannot "
        "verify the thrust estimator is off";
      if (why != precondition_) {status("REFUSED: " + why);}   // say it once, keep asking
      precondition_ = why;
      est_request_t0_ = now;   // re-ask after the retry interval
    } else {
      precondition_.clear();
      status("/" + estimator_node_ + ": thrust estimator is off (verified)");
    }
    return !on;
  }
  if (pending_est_) {
    if (now - est_request_t0_ > service_timeout_) {
      estimator_param_cli_->remove_pending_request(est_req_id_);
      pending_est_.reset();
      const std::string why = "no answer from /" + estimator_node_ + "/get_parameters: "
        "cannot verify the thrust estimator is off (set estimator_node:='' only if this "
        "stack has no geometric_mavros node)";
      if (why != precondition_) {status("REFUSED: " + why);}
      precondition_ = why;
    }
    return false;
  }
  if (now - est_request_t0_ < 2.0 && est_request_t0_ > 0.0) {return false;}
  if (!estimator_param_cli_->service_is_ready()) {
    if (precondition_.empty()) {
      precondition_ = "waiting for /" + estimator_node_ + " to verify the thrust estimator is off";
      status(precondition_);
    }
    est_request_t0_ = now;
    return false;
  }
  auto req = std::make_shared<GetParameters::Request>();
  req->names = {"enable_thrust_estimator"};
  auto fut = estimator_param_cli_->async_send_request(req);
  est_req_id_ = fut.request_id;
  pending_est_ = fut.future.share();
  est_request_t0_ = now;
  return false;
}

void TuningConductor::cmd_cb(const mav_controllers_ros::msg::SE3Command::SharedPtr msg)
{
  last_cmd_force_ = {msg->force.x, msg->force.y, msg->force.z};
  last_cmd_t_ = now_s();
}

void TuningConductor::record_sample()
{
  const double now = now_s();
  // Not in UPDATE_GAINS: the new gains are recorded as in force before the
  // controller has confirmed them.
  const bool flying = is_active_state(state_) && state_ != TunerState::TOO_LOW &&
    state_ != TunerState::UPDATE_GAINS;
  const bool fresh = sched_.odom && last_cmd_force_ && (now - last_cmd_t_) < 0.1 &&
    (now - sched_.odom->t) < 0.1;
  if (!flying || !fresh || !gains_ || controller_mass_ <= 0.0) {
    rec_gap_ = true;
    return;
  }
  if (rec_gap_ && !recording_.samples.empty()) {++rec_seg_;}
  rec_gap_ = false;
  SessionSample smp;
  smp.t = session_t0_ >= 0.0 ? now - session_t0_ : 0.0;
  smp.seg = rec_seg_;
  const auto & f = *last_cmd_force_;
  for (int i = 0; i < 3; ++i) {
    const std::string ax(1, "xyz"[i]);
    smp.r[i] = sched_.setpoint[i];
    smp.v[i] = sched_.odom->vel[i];
    smp.u[i] = f[i] / controller_mass_ - (i == 2 ? kGravity : 0.0);
    smp.kx[i] = gains_->at(ax).first;
    smp.kv[i] = gains_->at(ax).second;
  }
  recording_.samples.push_back(smp);
}

void TuningConductor::save_session_recording()
{
  if (recording_.samples.empty()) {return;}
  std::string dir = output_dir_;
  if (dir.empty()) {dir = std::filesystem::path(report_path_).parent_path().string();}
  if (dir.empty()) {return;}
  const auto path = std::filesystem::path(dir) /
    ("geo_tuner_session_" + (session_stamp_.empty() ? std::string("unknown") : session_stamp_) +
    ".csv");
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (recording_.save_csv(path.string())) {
    session_csv_ = path.string();
  } else {
    RCLCPP_WARN(get_logger(), "could not write session recording %s", path.string().c_str());
  }
}

void TuningConductor::st_wait_offboard(double)
{
  // Track the vehicle's current pose -- position AND heading -- so that
  // the setpoint the controller picks up the instant OFFBOARD engages is
  // the one it is already holding. Tracking position but not yaw still
  // commands a snap to north on handover.
  if (sched_.odom) {
    sched_.setpoint = sched_.odom->pos;
    sched_.setpoint_yaw = TuningSchedule::yaw_of(sched_.odom->quat);
  }
  if (in_offboard()) {
    status(
      offboard_mode_ + " engaged; resuming (round " + std::to_string(round_ + 1) + "/" +
      std::to_string(max_rounds_) + ", axis " + sched_.axes[sched_.axis_idx] + ")");
    begin_flying();
  }
}

void TuningConductor::st_too_low(double now)
{
  // Holding where the pilot handed over, because that was below the
  // working altitude. Nothing is commanded to move: the way out is the
  // pilot climbing (which needs manual control, so the mode change back
  // to OFFBOARD re-enters here) or the operator lowering the gate.
  auto [ok, why] = altitude_gate();
  if (ok) {
    why = saturation_gate();
    ok = why.empty();
  }
  if (ok) {
    capture_hover();
    status(
      "Altitude gate satisfied (" + fmt(hover_agl().value_or(0.0), 1) +
      " m AGL); starting");
    goto_state(TunerState::GOTO_HOVER);
    return;
  }
  if (now - too_low_log_t_ > 5.0) {
    too_low_log_t_ = now;
    status(why, false);
  }
}

void TuningConductor::st_goto_hover(double now)
{
  if (!sched_.odom) {return;}
  // Walk the setpoint to the hover point at a bounded speed. In capture
  // mode it is already there and this does nothing; in fixed mode it is
  // the difference between a commanded flight and a step the size of the
  // whole distance, which saturates the controller at max_accel.
  const double dt = 1.0 / std::max(rate_hz_, 1.0);
  if (goto_t0_ != state_t0_) {
    goto_t0_ = state_t0_;
    double d = 0.0;
    for (int i = 0; i < 3; ++i) {
      const double e = hover_pt_[i] - sched_.setpoint[i];
      d += e * e;
    }
    goto_eta_ = std::sqrt(d) / std::max(hover_speed_, 1e-3) +
      std::abs(wrap_angle(hover_yaw_ - sched_.setpoint_yaw)) /
      std::max(hover_yaw_rate_, 1e-3);
  }
  sched_.setpoint = ramp_toward(sched_.setpoint, hover_pt_, hover_speed_ * dt);
  sched_.setpoint_yaw = sched_.setpoint_yaw + ramp_toward(
    0.0, wrap_angle(hover_yaw_ - sched_.setpoint_yaw), hover_yaw_rate_ * dt);
  const bool ramp_done =
    std::abs(sched_.setpoint[0] - hover_pt_[0]) < 1e-6 &&
    std::abs(sched_.setpoint[1] - hover_pt_[1]) < 1e-6 &&
    std::abs(sched_.setpoint[2] - hover_pt_[2]) < 1e-6;
  double sq = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double d = sched_.odom->pos[i] - sched_.setpoint[i];
    sq += d * d;
  }
  const double err = std::sqrt(sq);
  const double speed = std::sqrt(
    sched_.odom->vel[0] * sched_.odom->vel[0] +
    sched_.odom->vel[1] * sched_.odom->vel[1] +
    sched_.odom->vel[2] * sched_.odom->vel[2]);
  if (ramp_done && err < hover_capture_radius_ && speed < hover_capture_speed_) {
    status(
      "At hover point; settling (<= " + fmt(settle_time_, 1) + "s) (round " +
      std::to_string(round_ + 1) + "/" + std::to_string(max_rounds_) +
      ", axis " + sched_.axes[sched_.axis_idx] + ")");
    goto_state(TunerState::SETTLE);
    return;
  }
  // Steady offset (velocity small, error persistent): absorb it into the
  // acceleration feedforward trim, the bounded stand-in for integral
  // action. The residual force each axis is missing equals kx * offset
  // (that's what the feedback is currently supplying).
  if (ramp_done && now - state_t0_ > 5.0 && speed < 0.3 && gains_) {
    bool updated = false;
    for (const auto & ax : {std::string("x"), std::string("y"), std::string("z")}) {
      const int i = axis_index(ax);
      const double off = sched_.setpoint[i] - sched_.odom->pos[i];
      if (std::abs(off) > 0.15) {
        const double kx = gains_->at(ax).first;
        a_trim_[i] = std::clamp(a_trim_[i] + 0.8 * kx * off, -max_trim_, max_trim_);
        updated = true;
      }
    }
    if (updated) {
      ++trim_updates_;
      status(
        "steady offset -> accel trim [" + fmt(a_trim_[0], 2) + ", " +
        fmt(a_trim_[1], 2) + ", " + fmt(a_trim_[2], 2) + "] m/s^2 (" +
        std::to_string(trim_updates_) + "/" + std::to_string(max_trim_updates_) + ")");
      const bool saturated = std::any_of(
        a_trim_.begin(), a_trim_.end(),
        [this](double a) {return std::abs(a) >= max_trim_;});
      if (trim_updates_ > max_trim_updates_ || saturated) {
        set_trim_diagnosis();
        abort(
          "steady-state offset exceeds trim authority" +
          (diagnosis_.empty() ? std::string() : "; " + diagnosis_));
        return;
      }
      state_t0_ = now;  // give the trim time to act
      return;
    }
  }
  if (now - state_t0_ > hover_timeout_ + goto_eta_) {
    set_trim_diagnosis();
    abort(
      "could not reach hover point in " + fmt(hover_timeout_ + goto_eta_, 0) + "s" +
      (diagnosis_.empty() ? std::string() : "; " + diagnosis_));
  }
}

void TuningConductor::set_trim_diagnosis()
{
  // A persistent z trim maps 1:1 to a thrust-map (max_thrust) error.
  const double az = a_trim_[2];
  if (std::abs(az) > 0.15) {
    const double alpha = 9.81 / (9.81 + az);
    diagnosis_ =
      "z accel trim " + fmt(az, 2) + " m/s^2 implies thrust-map scale ~" +
      fmt(alpha, 2) + ": multiply max_thrust in geometric_mavros.yaml by " +
      fmt(alpha, 2);
  }
}

std::pair<double, double> TuningConductor::command_limits() const
{
  // The controller clamps the feedback acceleration's norm to max_accel and
  // its tilt to max_tilt_angle; near hover the tilt allows g*tan(tilt)
  // laterally. PX4 and the motors saturate beyond that, where the linear
  // loop the identification assumes stops holding.
  double lat = ctrl_max_accel_;
  if (ctrl_max_tilt_ > 0.0 && ctrl_max_tilt_ < 1.5) {
    const double tilt_lat = kGravity * std::tan(ctrl_max_tilt_);
    lat = lat > 0.0 ? std::min(lat, tilt_lat) : tilt_lat;
  }
  return {lat, ctrl_max_accel_};
}

std::string TuningConductor::saturation_gate() const
{
  for (const auto & [label, vertical, mag] :
    {std::tuple<const char *, bool, double>{"step_size", false, sched_.step_size},
      std::tuple<const char *, bool, double>{"step_size_z", true, sched_.step_size_z}})
  {
    const auto why = step_saturation_reason(mag, vertical);
    if (!why.empty()) {return std::string("refusing to start: ") + label + " " + fmt(mag, 2) +
             " m -- " + why;}
  }
  return "";
}

std::string TuningConductor::step_saturation_reason(double step, bool vertical) const
{
  if (saturation_margin_ <= 0.0 || !gains_) {return "";}
  const auto [lat, vert] = command_limits();
  const double limit = vertical ? vert : lat;
  if (limit <= 0.0) {return "";}
  double kx = 0.0;
  for (const auto & ax : vertical ? std::vector<std::string>{"z"} :
    std::vector<std::string>{"x", "y"})
  {
    kx = std::max(kx, gains_->at(ax).first);
  }
  // A position step's command starts at kx * step and only falls from there.
  const double peak = kx * std::abs(step);
  if (peak <= saturation_margin_ * limit) {return "";}
  return "the step commands ~" + fmt(peak, 1) + " m/s^2 at kx " + fmt(kx, 2) +
         ", above " + fmt(100.0 * saturation_margin_, 0) + " % of the controller's " +
         fmt(limit, 1) + " m/s^2 " + (vertical ? "acceleration" : "lateral") +
         " limit; past it the loop is not the linear one being identified. Use <= " +
         fmt(saturation_margin_ * limit / kx, 2) + " m";
}

void TuningConductor::st_settle(double now)
{
  const double elapsed = now - state_t0_;
  // Both predicates track their own windows: evaluate each once per tick.
  const bool quiet = sched_.is_quiet(now);
  const bool parked = sched_.is_parked(now);
  if (elapsed < min_settle_time_) {return;}
  // A parked vehicle with a steady offset: absorb it into the accel trim
  // BEFORE stepping, then re-settle under the new trim so the step starts
  // from a trimmed equilibrium (T4 in TUNER_IMPROVEMENTS_PLAN.md). The
  // residual force each axis is missing equals kx * offset: that is what
  // the feedback is currently supplying. Gated on PARKED (holding still
  // wherever it is), not quiet: quiet demands closeness to the setpoint, so
  // it refused precisely the offsets this exists to remove. A gust keeps the
  // vehicle moving and fails the parked test, so it is not learned as trim.
  if (parked && sched_.odom && gains_ && trim_updates_ < max_trim_updates_) {
    bool updated = false;
    for (const auto & ax2 : {std::string("x"), std::string("y"), std::string("z")}) {
      const int i = axis_index(ax2);
      const double off = sched_.setpoint[i] - sched_.odom->pos[i];
      if (std::abs(off) > trim_update_threshold_) {
        const double kx = gains_->at(ax2).first;
        a_trim_[i] = std::clamp(a_trim_[i] + 0.8 * kx * off, -max_trim_, max_trim_);
        updated = true;
      }
    }
    if (updated) {
      ++trim_updates_;
      status(
        "steady offset -> accel trim [" + fmt(a_trim_[0], 2) + ", " +
        fmt(a_trim_[1], 2) + ", " + fmt(a_trim_[2], 2) + "] m/s^2 (" +
        std::to_string(trim_updates_) + "/" + std::to_string(max_trim_updates_) + ")");
      if (std::any_of(
          a_trim_.begin(), a_trim_.end(),
          [this](double a) {return std::abs(a) >= max_trim_;}))
      {
        set_trim_diagnosis();
        abort(
          "steady-state offset exceeds trim authority" +
          (diagnosis_.empty() ? std::string() : "; " + diagnosis_));
        return;
      }
      // Unlike GOTO_HOVER, an exhausted update budget here is not an
      // abort: the trim simply freezes and the session continues with
      // whatever it has learned.
      state_t0_ = now;   // re-settle under the new trim before stepping
      sched_.reset_quiet();
      return;
    }
  }
  // settle_time remains the hard cap: a windy vehicle that never goes quiet
  // steps anyway.
  if (elapsed < settle_time_ && !quiet) {return;}
  last_settle_dur_ = elapsed;
  const std::string ax = sched_.axes[sched_.axis_idx];
  sched_.recording.clear();
  // Same clock as the samples that will be recorded against it.
  sched_.step_t0 = sched_.odom ? sched_.odom->t_stamp : now;
  sched_.reset_quiet();
  double wn_equiv = 0.0;
  double step = 0.0;
  if (ax == "yaw") {
    const double new_off = sched_.next_leg(sched_.yaw_step);
    step = new_off - sched_.leg_offset;
    sched_.leg_offset = new_off;
    pre_step_yaw_ = TuningSchedule::yaw_of(sched_.odom->quat);
    sched_.setpoint = hover_pt_;
    sched_.setpoint_yaw = wrap_angle(hover_yaw_ + new_off);
    wn_equiv = 1.0 / std::max(yaw_T_target_, 1e-3);
    status("Yaw step " + fmt(step, 2) + " rad");
  } else {
    const int i = axis_index(ax);
    const double mag = ax == "z" ? sched_.step_size_z : sched_.step_size;
    double new_off = sched_.next_leg(mag);
    // A downward z leg is the one manoeuvre that flies at the ground. If
    // the leg the schedule wants would put the commanded point (plus the
    // undershoot it will overshoot to) under the safety floor, fly the
    // opposite leg instead: the identification only needs a step, not a
    // particular sign, and the alternative is descending into the floor
    // and aborting there.
    if (ax == "z" && new_off < 0.0) {
      const auto agl = hover_agl();
      if (agl && !z_leg_clears_floor(
          *agl, new_off, sched_.step_size_z, z_step_margin_,
          safety_.limits().min_altitude))
      {
        sched_.step_sign *= -1.0;
        new_off = sched_.next_leg(mag);
        status(
          "down step would come within " +
          fmt(safety_.limits().min_altitude, 1) + " m of the ground (" +
          fmt(*agl, 1) + " m AGL); flying the up leg instead");
      }
    }
    step = new_off - sched_.leg_offset;
    sched_.leg_offset = new_off;
    pre_step_pos_ = sched_.odom->pos[i];
    sched_.setpoint = hover_pt_;
    sched_.setpoint[i] += new_off;
    wn_equiv = std::sqrt(std::max(gains_->at(ax).first, 1e-3));
    status("Step " + fmt(step, 2) + " m on " + ax);
  }
  sched_.step_applied = step;
  // Earliest an adaptive episode may end: the transient of the currently
  // applied loop cannot have finished before this.
  episode_min_time_ = std::min(
    episode_time_,
    std::max(
      min_episode_time_,
      episode_settle_periods_ / std::max(zeta_target_ * wn_equiv, 1e-3)));
  ep_u_peak_lat_ = 0.0;
  ep_u_peak_z_ = 0.0;
  goto_state(TunerState::STEP);
}

void TuningConductor::st_step(double now)
{
  const std::string ax = sched_.axes[sched_.axis_idx];
  if (sched_.odom) {
    // One recording entry per odometry sample, on the sample's own clock;
    // the tick timer merely polls, so a tick without fresh odometry must
    // not duplicate the previous point.
    const double ts = sched_.odom->t_stamp - sched_.step_t0;
    if (sched_.recording.empty() || ts > sched_.recording.back().first) {
      if (ax == "yaw") {
        double dyaw = TuningSchedule::yaw_of(sched_.odom->quat) - pre_step_yaw_;
        dyaw = std::atan2(std::sin(dyaw), std::cos(dyaw));   // unwrap
        sched_.recording.emplace_back(ts, dyaw);
      } else {
        const int i = axis_index(ax);
        sched_.recording.emplace_back(ts, sched_.odom->pos[i] - pre_step_pos_);
      }
    }
  }
  const double elapsed = now - state_t0_;
  if (elapsed >= episode_time_) {
    goto_state(TunerState::ANALYZE);
    return;
  }
  if (last_cmd_force_ && controller_mass_ > 0.0) {
    const auto & f = *last_cmd_force_;
    const double ux = f[0] / controller_mass_, uy = f[1] / controller_mass_;
    ep_u_peak_lat_ = std::max(ep_u_peak_lat_, std::hypot(ux, uy));
    ep_u_peak_z_ = std::max(ep_u_peak_z_, std::abs(f[2] / controller_mass_ - kGravity));
  }
  if (!adaptive_episode_ || elapsed < episode_min_time_ || !sched_.odom) {return;}
  const bool ended = (ax != "yaw" && episode_end_ == "motion") ?
    sched_.motion_quiet(now) : sched_.response_settled(sched_.odom->t_stamp);
  if (ended) {
    goto_state(TunerState::ANALYZE);
  }
}

void TuningConductor::dump_episode(
  const std::string & ax, const Eigen::VectorXd & t,
  const Eigen::VectorXd & y, double step)
{
  const std::string dir = !output_dir_.empty() ?
    (std::filesystem::path(output_dir_) / ("episodes_" + session_stamp_)).string() :
    episode_dump_dir_;
  if (dir.empty()) {return;}
  try {
    std::filesystem::create_directories(dir);
    std::ostringstream name;
    name << "ep" << std::setw(3) << std::setfill('0') << episode_seq_++ <<
      "_" << ax << "_round" << round_ << "_rep" << sched_.rep << ".csv";
    std::ofstream f(std::filesystem::path(dir) / name.str());
    f << "# axis=" << ax << " step=" << step << " round=" << round_ <<
      " rep=" << sched_.rep << "\n";
    f << "t,y\n" << std::setprecision(9);
    for (Eigen::Index i = 0; i < t.size(); ++i) {
      f << t[i] << "," << y[i] << "\n";
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "episode dump failed: %s", e.what());
  }
}

void TuningConductor::analyze_yaw()
{
  Eigen::VectorXd t(sched_.recording.size()), y(sched_.recording.size());
  for (size_t i = 0; i < sched_.recording.size(); ++i) {
    t[static_cast<Eigen::Index>(i)] = sched_.recording[i].first;
    y[static_cast<Eigen::Index>(i)] = sched_.recording[i].second;
  }
  const double step = sched_.step_applied;
  dump_episode("yaw", t, y, step);
  YAML::Node rec;
  rec["axis"] = "yaw";
  rec["round"] = static_cast<int>(round_);
  rec["rep"] = sched_.rep;
  rec["T_target"] = yaml_double(yaw_T_target_);
  rec["step"] = yaml_double(round_to(step, 3));
  rec["yaw_tau_applied"] = yaml_double(round_to(*yaw_tau_, 3));
  rec["settle_s"] = yaml_double(round_to(last_settle_dur_, 2));
  rec["record_s"] = yaml_double(round_to(t.size() > 0 ? t[t.size() - 1] : 0.0, 2));

  FirstOrderFitResult fit;
  try {
    fit = fit_first_order(t, y, step, std::max(*yaw_tau_ / 2.0, 0.05));
  } catch (const std::exception & e) {
    rec["action"] = std::string("episode discarded (fit error: ") + e.what() + ")";
    results_.push_back(rec);
    status(std::string("yaw fit failed (") + e.what() + "); discarding episode");
    episode_finished();
    return;
  }
  rec["T_meas"] = yaml_double(round_to(fit.T, 3));
  rec["delay"] = yaml_double(round_to(fit.delay, 3));
  rec["nrmse"] = yaml_double(round_to(fit.nrmse, 3));
  status(
    "yaw: T=" + fmt(fit.T, 2) + "s delay=" + fmt(fit.delay * 1e3, 0) +
    "ms nrmse=" + fmt(fit.nrmse, 2));
  // Gate on the yaw-specific nrmse screen, not FirstOrderFitResult::ok():
  // that hardcodes the position loops' 0.15 (see yaw_fit_nrmse above).
  if (!(fit.converged && !fit.at_bounds && fit.nrmse < yaw_fit_nrmse_)) {
    rec["action"] = "fit rejected";
    status(
      "Yaw fit quality gate failed (nrmse " + fmt(fit.nrmse, 2) + " vs " +
      fmt(yaw_fit_nrmse_, 2) + ", converged=" + (fit.converged ? "y" : "n") +
      ", at_bounds=" + (fit.at_bounds ? "y" : "n") + "); episode discarded");
  } else {
    rec["action"] = "accepted";
    axis_prog_["yaw"].yaw_T.push_back(fit.T);
  }
  results_.push_back(rec);
  episode_finished();
}

void TuningConductor::st_analyze(double)
{
  const std::string ax = sched_.axes[sched_.axis_idx];
  if (ax == "yaw") {
    analyze_yaw();
    return;
  }
  // No per-episode fitting: one step cannot separate plant gain from lag
  // against a gust (see core/accel_loop_id.hpp). The episode is excitation
  // for the round-end identification, which reads the session recording;
  // here only what the record itself shows is kept -- the overshoot, which
  // validation checks.
  const double step = sched_.step_applied;
  Eigen::VectorXd t(sched_.recording.size()), y(sched_.recording.size());
  for (size_t i = 0; i < sched_.recording.size(); ++i) {
    t[static_cast<Eigen::Index>(i)] = sched_.recording[i].first;
    y[static_cast<Eigen::Index>(i)] = sched_.recording[i].second;
  }
  dump_episode(ax, t, y, step);
  const double os = measured_overshoot(sched_.recording, step);
  YAML::Node rec;
  rec["axis"] = ax;
  rec["round"] = static_cast<int>(round_);
  rec["rep"] = sched_.rep;
  rec["step"] = yaml_double(step);
  rec["kx_applied"] = yaml_double(round_to(gains_->at(ax).first, 3));
  rec["kv_applied"] = yaml_double(round_to(gains_->at(ax).second, 3));
  rec["overshoot"] = yaml_double(round_to(os, 3));
  rec["settle_s"] = yaml_double(round_to(last_settle_dur_, 2));
  rec["record_s"] = yaml_double(round_to(t.size() > 0 ? t[t.size() - 1] : 0.0, 2));
  const bool vertical = ax == "z";
  const double u_peak = vertical ? ep_u_peak_z_ : ep_u_peak_lat_;
  rec["u_peak"] = yaml_double(round_to(u_peak, 2));
  const auto limits = command_limits();
  const double u_limit = vertical ? limits.second : limits.first;
  std::string sat_note;
  if (u_limit > 0.0) {
    rec["u_peak_frac"] = yaml_double(round_to(u_peak / u_limit, 2));
    if (saturation_margin_ > 0.0 && u_peak > saturation_margin_ * u_limit) {
      rec["near_saturation"] = true;
      sat_note = "; WARNING peak command " + fmt(u_peak, 1) + " m/s^2 is " +
        fmt(100.0 * u_peak / u_limit, 0) + " % of the " + fmt(u_limit, 1) +
        " m/s^2 limit -- consider a smaller step";
    }
  }
  rec["action"] = "recorded";
  results_.push_back(rec);
  axis_prog_[ax].overshoots.push_back(os);
  status(ax + ": step " + fmt(step, 2) + " flown, overshoot " + fmt(100.0 * os, 0) + "%" +
    sat_note);
  episode_finished();
}

void TuningConductor::decide_position_axis(const std::string & ax, Gains & to_apply)
{
  auto & pr = axis_prog_[ax];
  const double kx_now = gains_->at(ax).first;
  const double kv_now = gains_->at(ax).second;
  DecisionConfig dc = decision_;
  if (ax == "z") {dc.wn_target = wn_target_z_;}
  const bool rounds_left = round_ + 1 < static_cast<size_t>(max_rounds_);

  YAML::Node rec;
  rec["axis"] = ax;
  rec["round"] = static_cast<int>(round_);
  rec["kx_in_force"] = yaml_double(round_to(kx_now, 3));
  rec["kv_in_force"] = yaml_double(round_to(kv_now, 3));
  const auto put_id = [](YAML::Node & n, const AccelLoopResult & id) {
      n["ok"] = id.ok;
      if (!id.ok) {n["reason"] = id.reason;}
      n["alpha"] = yaml_double(round_to(id.alpha, 3));
      n["alpha_ci"] = std::vector<double>{round_to(id.alpha_lo, 3), round_to(id.alpha_hi, 3)};
      n["lag_ms"] = yaml_double(round_to(1e3 * id.lag, 0));
      n["lag_ci_ms"] = std::vector<double>{round_to(1e3 * id.lag_lo, 0),
        round_to(1e3 * id.lag_hi, 0)};
      n["delay_ms"] = yaml_double(round_to(1e3 * id.delay, 0));
      n["tau_ms"] = yaml_double(round_to(1e3 * id.tau, 0));
      n["r2"] = yaml_double(round_to(id.r2, 3));
      n["r2_raw"] = yaml_double(round_to(id.r2_raw, 3));
      n["excited_s"] = yaml_double(round_to(id.excited_s, 1));
    };

  // 1. Gains applied last round: accept or restore, on THIS round's data.
  if (pr.pending_validation) {
    pr.pending_validation = false;
    const auto id_val = round_ids_[ax].validation;
    const double os = pr.overshoots.empty() ? 0.0 : median(pr.overshoots);
    const auto v = validate_gains(
      kx_now, kv_now, id_val, pr.id_design, os, dc.margins, max_overshoot_);
    YAML::Node vn;
    put_id(vn, id_val);
    vn["overshoot_median"] = yaml_double(round_to(os, 3));
    vn["pm_nominal_deg"] = yaml_double(round_to(v.pm_nominal_deg, 1));
    vn["pm_worst_deg"] = yaml_double(round_to(v.pm_worst_deg, 1));
    vn["pass"] = v.pass;
    vn["why"] = v.why;
    rec["validation"] = vn;
    if (!v.pass) {
      const auto entry = baseline_gains_->at(ax);
      to_apply[ax] = entry;
      (*gains_)[ax] = entry;
      (*safe_gains_)[ax] = entry;
      pr.active = false;
      pr.result = "restored";
      pr.outcome = "validation failed (" + v.why + "); entry gains restored";
      rec["action"] = pr.outcome;
      results_.push_back(rec);
      session_result_[ax] = pr.outcome;
      status(ax + ": " + pr.outcome);
      return;
    }
    pr.validated = true;
    (*safe_gains_)[ax] = gains_->at(ax);
    status(ax + ": applied gains " + v.why);
  }

  // 2. The plant does not depend on the gains: identify from everything.
  const auto id = round_ids_[ax].all;
  pr.id_last = id;
  const auto dec = decide_axis(kx_now, kv_now, id, dc);
  YAML::Node idn;
  put_id(idn, id);
  rec["identification"] = idn;
  rec["verdict"] = to_string(dec.verdict);
  rec["why"] = dec.why;
  if (dec.verdict != AxisVerdict::NO_ESTIMATE) {
    rec["pm_now_nominal_deg"] = yaml_double(round_to(dec.pm_now_nominal_deg, 1));
    rec["pm_now_worst_deg"] = yaml_double(round_to(dec.pm_now_worst_deg, 1));
    YAML::Node dn;
    dn["wn"] = yaml_double(round_to(dec.design.wn, 3));
    dn["kx"] = yaml_double(round_to(dec.design.kx, 3));
    dn["kv"] = yaml_double(round_to(dec.design.kv, 3));
    dn["pm_nominal_deg"] = yaml_double(round_to(dec.design.pm_nominal_deg, 1));
    dn["pm_worst_deg"] = yaml_double(round_to(dec.design.pm_worst_deg, 1));
    dn["wn_limited_by_margin"] = dec.design.limited;
    rec["design"] = dn;
  }
  status(
    ax + ": alpha " + fmt(id.alpha, 2) + " [" + fmt(id.alpha_lo, 2) + ", " +
    fmt(id.alpha_hi, 2) + "], lag " + fmt(1e3 * id.lag, 0) + " ms [" + fmt(1e3 * id.lag_lo, 0) +
    ", " + fmt(1e3 * id.lag_hi, 0) + "] -> " + dec.why);

  const std::string prefix = pr.validated ? "validated; " : "";
  switch (dec.verdict) {
    case AxisVerdict::INCONCLUSIVE:
      if (rounds_left) {
        pr.outcome = "inconclusive; flying another round to narrow the interval";
      } else {
        pr.active = false;
        pr.result = pr.validated ? "updated_validated" : "kept_inconclusive";
        pr.outcome = prefix + "kept, " + dec.why;
      }
      break;
    case AxisVerdict::NO_ESTIMATE:
      if (rounds_left) {
        pr.outcome = "no estimate yet (" + id.reason + "); flying another round";
      } else {
        pr.active = false;
        pr.result = pr.validated ? "updated_validated" : "kept_no_estimate";
        pr.outcome = prefix + "kept: " + dec.why;
      }
      break;
    case AxisVerdict::CONFIRMED:
      pr.active = false;
      pr.result = pr.validated ? "updated_validated" : "confirmed";
      pr.outcome = pr.validated ? "validated: " + dec.why : dec.why;
      break;
    case AxisVerdict::UPDATE:
      if (rounds_left) {
        // max_gain_change_factor bounds the whole session against the gains
        // it started from, not each round against the last: per round it
        // compounds (1.6^2 = 2.56x over three rounds).
        const auto entry = baseline_gains_->at(ax);
        const double f = dc.max_change;
        const double kx_new = std::clamp(dec.kx_new, entry.first / f, entry.first * f);
        const double kv_new = std::clamp(dec.kv_new, entry.second / f, entry.second * f);
        const bool at_limit = kx_new != dec.kx_new || kv_new != dec.kv_new;
        to_apply[ax] = {kx_new, kv_new};
        (*gains_)[ax] = {kx_new, kv_new};
        pr.pending_validation = true;
        pr.id_design = id;
        pr.outcome = "applied kx " + fmt(kx_new, 3) + " kv " + fmt(kv_new, 3) +
          (at_limit ? " (at the session change limit " + fmt(f, 2) + "x of entry gains)" : "") +
          ", validating next round";
      } else {
        pr.active = false;
        pr.result = pr.validated ? "updated_validated" : "kept_update_unvalidated";
        pr.outcome = prefix + "update indicated but no round left to validate it; gains kept (" +
          dec.why + ")";
      }
      break;
  }
  rec["action"] = pr.outcome;
  results_.push_back(rec);
  session_result_[ax] = pr.outcome;
}

bool TuningConductor::decide_yaw()
{
  auto & pr = axis_prog_["yaw"];
  const bool rounds_left = round_ + 1 < static_cast<size_t>(max_rounds_);
  const auto est = robust_ratio_estimate(
    pr.yaw_T, std::min(2, sched_.episodes_per_rung), consistency_);
  YAML::Node rec;
  rec["axis"] = "yaw";
  rec["round"] = static_cast<int>(round_);
  rec["n_used"] = est.n_used;
  rec["yaw_tau_in_force"] = yaml_double(round_to(*yaw_tau_, 3));
  if (std::isfinite(est.spread)) {rec["spread"] = yaml_double(round_to(est.spread, 3));}
  const auto log_err = [this](double T) {return std::abs(std::log(T / yaw_T_target_));};
  const double tol = std::log(1.0 + yaw_tolerance_);

  if (pr.pending_validation) {
    pr.pending_validation = false;
    // The applied tau must have moved T toward the target (or into the band).
    const bool pass = est.ok &&
      (log_err(est.value) <= tol || log_err(est.value) < log_err(pr.yaw_T_design));
    rec["validation"] = pass ? "pass" : "fail";
    if (!pass) {
      yaw_tau_ = baseline_yaw_tau_;
      pr.active = false;
      pr.result = "restored";
      pr.outcome = std::string("validation failed (") +
        (est.ok ? "T " + fmt(est.value, 2) + " s no closer to target " +
        fmt(yaw_T_target_, 2) + " than before" : est.reason) +
        "); entry yawctrl_tau restored";
      rec["action"] = pr.outcome;
      results_.push_back(rec);
      session_result_["yaw"] = pr.outcome;
      status("yaw: " + pr.outcome);
      return true;
    }
    pr.validated = true;
    safe_yaw_tau_ = yaw_tau_;
  }
  if (!est.ok) {
    if (rounds_left) {
      pr.outcome = "no estimate yet (" + est.reason + ")";
    } else {
      pr.active = false;
      pr.result = pr.validated ? "updated_validated" : "kept_no_estimate";
      pr.outcome = "kept: " + est.reason;
    }
    rec["action"] = pr.outcome;
    results_.push_back(rec);
    session_result_["yaw"] = pr.outcome;
    return false;
  }
  rec["T_median"] = yaml_double(round_to(est.value, 3));
  bool applied = false;
  if (log_err(est.value) <= tol) {
    pr.active = false;
    pr.result = pr.validated ? "updated_validated" : "confirmed";
    pr.outcome = std::string(pr.validated ? "validated" : "confirmed") + ": T " +
      fmt(est.value, 2) + " s within " + fmt(100.0 * yaw_tolerance_, 0) + " % of " +
      fmt(yaw_T_target_, 2) + " s";
  } else if (rounds_left) {
    double tau_new = *yaw_tau_ * yaw_T_target_ / est.value;
    tau_new = std::clamp(tau_new, *yaw_tau_ / max_change_, *yaw_tau_ * max_change_);
    tau_new = std::clamp(tau_new, yaw_tau_min_, yaw_tau_max_);
    pr.pending_validation = true;
    pr.yaw_T_design = est.value;
    pr.outcome = "T " + fmt(est.value, 2) + " s -> yawctrl_tau " + fmt(*yaw_tau_, 3) + " -> " +
      fmt(tau_new, 3) + ", validating next round";
    yaw_tau_ = tau_new;
    rec["yaw_tau_new"] = yaml_double(round_to(tau_new, 3));
    applied = true;
  } else {
    pr.active = false;
    pr.result = pr.validated ? "updated_validated" : "kept_update_unvalidated";
    pr.outcome = "T " + fmt(est.value, 2) + " s outside the band but no round left to "
      "validate an update; yawctrl_tau kept";
  }
  rec["action"] = pr.outcome;
  results_.push_back(rec);
  session_result_["yaw"] = pr.outcome;
  status("yaw: " + pr.outcome);
  return applied;
}

void TuningConductor::finish_round()
{
  // Snap the setpoint home before the (short) decision/parameter phase.
  sched_.setpoint = hover_pt_;
  sched_.setpoint_yaw = hover_yaw_;
  sched_.leg_offset = 0.0;
  status("round " + std::to_string(round_ + 1) + "/" + std::to_string(max_rounds_) +
    " flown; identifying");

  save_session_recording();   // survives the node dying mid-session

  // Snapshot the inputs on this thread; the recording keeps growing.
  struct Job
  {
    bool validate;
    std::vector<AxisSegment> round, all;
  };
  std::map<std::string, Job> jobs;
  for (const auto & ax : sched_.axes) {
    if (ax == "yaw" || !axis_prog_[ax].active) {continue;}
    const int i = axis_index(ax);
    Job j;
    j.validate = axis_prog_[ax].pending_validation;
    if (j.validate) {j.round = recording_.segments(i, round_first_sample_);}
    j.all = recording_.segments(i, 0);
    jobs[ax] = std::move(j);
  }
  id_future_ = std::async(
    std::launch::async, [jobs = std::move(jobs), cfg = id_cfg_]() {
      std::map<std::string, RoundIds> out;
      for (const auto & [ax, j] : jobs) {
        RoundIds r;
        if (j.validate) {r.validation = identify_accel_loop(j.round, cfg);}
        r.all = identify_accel_loop(j.all, cfg);
        out[ax] = r;
      }
      return out;
    });
  goto_state(TunerState::IDENTIFY);
}

void TuningConductor::st_identify(double now)
{
  // Holding the hover point meanwhile; safety and staleness keep running.
  if (id_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    if (now - state_t0_ > max_identify_s_) {
      abort("round identification did not finish in " + fmt(max_identify_s_, 0) + " s");
    }
    return;
  }
  round_ids_ = id_future_.get();
  status("identification took " + fmt(now - state_t0_, 2) + " s");
  conclude_round();
}

void TuningConductor::conclude_round()
{
  Gains to_apply;
  bool yaw_changed = false;
  for (const auto & ax : sched_.axes) {
    if (!axis_prog_[ax].active) {continue;}
    if (ax == "yaw") {
      yaw_changed = decide_yaw();
    } else {
      decide_position_axis(ax, to_apply);
    }
  }
  const bool any_active = std::any_of(
    axis_prog_.begin(), axis_prog_.end(), [](const auto & kv) {return kv.second.active;});
  if (!to_apply.empty() || yaw_changed) {
    finish_after_update_ = !any_active;
    apply_gains(to_apply, yaw_changed ? yaw_tau_ : std::nullopt);
    goto_state(TunerState::UPDATE_GAINS);
    return;
  }
  if (!any_active) {
    finish();
    return;
  }
  start_next_round();
}

void TuningConductor::start_next_round()
{
  ++round_;
  round_first_sample_ = recording_.size();
  for (auto & [ax, pr] : axis_prog_) {
    pr.overshoots.clear();
    pr.yaw_T.clear();
  }
  sched_.axis_idx = 0;
  sched_.rep = 0;
  sched_.leg_offset = 0.0;
  sched_.bucket = EpisodeBucket();
  sched_.setpoint = hover_pt_;
  sched_.setpoint_yaw = hover_yaw_;
  if (!seek_active_axis()) {return;}
  status("round " + std::to_string(round_ + 1) + "/" + std::to_string(max_rounds_));
  goto_state(TunerState::GOTO_HOVER);
}

void TuningConductor::st_update_gains(double now)
{
  if (!pending_set_ || !ready(*pending_set_)) {
    if (now - state_t0_ > 3.0) {
      abort("parameter update timed out");
    }
    return;
  }
  const auto res = pending_set_->get();
  pending_set_.reset();
  bool ok = res != nullptr;
  if (res) {
    for (const auto & r : res->results) {
      if (!r.successful) {ok = false;}
    }
  }
  if (!ok) {
    abort("controller rejected parameter update");
    return;
  }
  if (finish_after_update_) {
    finish_after_update_ = false;
    finish();
    return;
  }
  start_next_round();
}

void TuningConductor::fly_next_rep()
{
  if (sched_.bidirectional) {
    // Already settled-ish at the current leg; SETTLE waits for the
    // quiet window rather than flying a discarded return.
    goto_state(TunerState::SETTLE);
  } else {
    sched_.leg_offset = 0.0;
    sched_.setpoint = hover_pt_;
    sched_.setpoint_yaw = hover_yaw_;
    goto_state(TunerState::GOTO_HOVER);
  }
}

void TuningConductor::episode_finished()
{
  // Alternate step direction to stay centred on the hover point.
  // Bidirectional: flip only after the leg that returns to centre, giving
  // the 0 -> +d -> 0 -> -d -> 0 sequence.
  if (!sched_.bidirectional || sched_.leg_offset == 0.0) {
    sched_.step_sign *= -1.0;
  }
  ++sched_.rep;
  if (sched_.rep < sched_.episodes_per_rung) {
    fly_next_rep();
    return;
  }
  advance_axis();
}

void TuningConductor::advance_axis()
{
  if (sched_.advance_axis(hover_pt_, hover_yaw_)) {
    finish_round();
    return;
  }
  if (!seek_active_axis()) {return;}
  goto_state(TunerState::GOTO_HOVER);
}

bool TuningConductor::seek_active_axis()
{
  while (!axis_prog_[sched_.axes[sched_.axis_idx]].active) {
    if (sched_.advance_axis(hover_pt_, hover_yaw_)) {
      finish_round();
      return false;
    }
  }
  return true;
}

void TuningConductor::finish()
{
  sched_.setpoint = hover_pt_;
  sched_.setpoint_yaw = hover_yaw_;
  sched_.leg_offset = 0.0;
  set_trim_diagnosis();
  freeze_session_end();
  save_session_recording();
  write_report("complete");
  status("Tuning complete. Report: " + report_path_);
  goto_state(TunerState::DONE);
}

// ---- ground-station control ----

void TuningConductor::srv_start(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  if (state_ == TunerState::ABORT || state_ == TunerState::DONE) {
    res->success = false;
    res->message = std::string("Session already finished (") + to_string(state_) +
      "); call ~/reset to arm another one.";
    return;
  }
  if (state_ != TunerState::WAIT_ODOM && state_ != TunerState::WAIT_ENABLE &&
    state_ != TunerState::WAIT_OFFBOARD)
  {
    res->success = false;
    res->message = std::string("Already running (") + to_string(state_) + ").";
    return;
  }
  start_requested_ = true;
  res->success = true;
  res->message =
    "Start requested. The session begins once odometry, the baseline gains and "
    "OFFBOARD are all present.";
  status(res->message);
}

void TuningConductor::srv_abort(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  // Always allowed, in every state: an abort you cannot press is not an
  // abort. It restores the last known-safe gains and holds hover.
  if (state_ == TunerState::ABORT) {
    res->success = true;
    res->message = "Already aborted: " + abort_reason_ +
      ". Call ~/reset to arm another session.";
    return;
  }
  abort("aborted from the ground station");
  res->success = true;
  res->message = restore_state_ == "pending" ?
    "Aborted: holding position; restoring the last validated gains (see health 'restore')." :
    "Aborted: holding position; no gains had been changed.";
}

void TuningConductor::srv_accept(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  if (!gains_) {
    res->success = false;
    res->message = "No gains identified yet.";
    return;
  }
  // Accepting means the CURRENT gains become the ones an abort would
  // restore, so a later problem cannot silently undo a good result.
  safe_gains_ = gains_;
  write_report("accepted");
  res->success = true;
  res->message =
    "Accepted the current gains as the safe set. Report: " + report_path_ +
    ". They are live on the controller but not persisted -- use gain_saver to "
    "write them to the vehicle.";
  status(res->message);
}

void TuningConductor::srv_restore_safe(
  Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  if (!safe_gains_) {
    res->success = false;
    res->message = "No safe gain set captured yet.";
    return;
  }
  gains_ = safe_gains_;
  apply_gains(*gains_);
  res->success = true;
  res->message = "Restored the last known-safe gains.";
  status(res->message);
}

void TuningConductor::srv_reset(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res)
{
  // Return an aborted or finished session to the waiting state.
  //
  // Without this the only way to try again after an abort is to restart
  // the node -- which in the field means an SSH session, in flight, to
  // recover from a condition the tuner detected on purpose.
  if (state_ != TunerState::ABORT && state_ != TunerState::DONE) {
    res->success = false;
    res->message = std::string("Nothing to reset: session is ") + to_string(state_) +
      ". Abort first if you want to stop it.";
    return;
  }
  if (restore_state_ == "pending" || restore_state_ == "UNCONFIRMED") {
    res->success = false;
    res->message = "Gain restore is " + restore_state_ +
      ": a new session would start from unknown gains. Land and check the controller's gains.";
    return;
  }

  // Fly on whatever gains are currently in force: an abort has already
  // restored the safe set, and silently changing them here would hide
  // what the vehicle is actually using.
  abort_reason_.clear();
  diagnosis_.clear();
  results_.clear();
  sched_.recording.clear();
  sched_.bucket = EpisodeBucket();
  round_ = 0;
  round_first_sample_ = 0;
  recording_.clear();
  rec_seg_ = 0;
  rec_gap_ = true;
  session_csv_.clear();
  axis_prog_.clear();
  finish_after_update_ = false;
  restore_state_.clear();
  restore_tries_ = 0;
  // Re-verify the estimator: it may have been switched on since.
  estimator_off_.reset();
  pending_est_.reset();
  est_request_t0_ = 0.0;
  precondition_.clear();
  sched_.axis_idx = 0;
  sched_.rep = 0;
  sched_.leg_offset = 0.0;
  sched_.step_sign = 1.0;
  sched_.reset_quiet();
  // The next session captures its own hover point: after an abort the
  // vehicle is rarely where the last one started.
  hover_captured_ = false;
  goto_t0_ = -1.0;
  session_t0_ = -1.0;
  last_settle_dur_ = 0.0;
  a_trim_ = {0.0, 0.0, 0.0};
  session_end_.reset();
  trim_updates_ = 0;
  start_requested_ = !require_enable_;
  safety_.reset();
  hold_here();
  baseline_read_ = false;
  request_gains();
  goto_state(TunerState::WAIT_ENABLE);
  res->success = true;
  res->message =
    "Reset. Fix what caused the abort, then press START again. The gains in "
    "force are unchanged.";
  status(res->message);
}

// What the session is doing, in words a pilot who has never read this
// code can act on. The machine-readable state stays in the "state" key;
// this feeds the banner and the "phase" key.
static const char * phase_string(TunerState s)
{
  switch (s) {
    case TunerState::WAIT_ODOM: return "waiting for the vehicle's position feed";
    case TunerState::WAIT_ENABLE: return "ready - press START";
    case TunerState::WAIT_OFFBOARD: return "waiting for OFFBOARD handover";
    case TunerState::TOO_LOW: return "too low to tune";
    case TunerState::GOTO_HOVER: return "steadying at the tuning point";
    case TunerState::SETTLE: return "holding steady before the next test step";
    case TunerState::STEP: return "flying a test step";
    case TunerState::ANALYZE: return "analyzing the response";
    case TunerState::IDENTIFY: return "identifying the round";
    case TunerState::UPDATE_GAINS: return "applying updated gains";
    case TunerState::DONE: return "finished";
    case TunerState::ABORT: return "stopped";
  }
  return "";
}

void TuningConductor::publish_health()
{
  DiagnosticStatus st;
  st.name = "geo_tuner";
  st.hardware_id = get_name();

  if (state_ == TunerState::ABORT) {
    st.level = DiagnosticStatus::ERROR;
    st.message = "aborted: " + abort_reason_;
  } else if (state_ == TunerState::DONE) {
    st.level = DiagnosticStatus::OK;
    st.message = "tuning finished";
  } else if (state_ == TunerState::TOO_LOW) {
    // Active (it is holding the vehicle) but not tuning, and the operator
    // has to do something about it: warn, and say what.
    st.level = DiagnosticStatus::WARN;
    st.message = "refused: " + altitude_gate().second;
  } else if (is_active_state(state_)) {
    st.level = DiagnosticStatus::OK;
    st.message = phase_string(state_);
  } else {
    st.level = DiagnosticStatus::WARN;
    st.message = phase_string(state_);
  }

  auto add = [&st](const std::string & key, const std::string & value) {
      KeyValue kv;
      kv.key = key;
      kv.value = value;
      st.values.push_back(kv);
    };

  const std::string axis =
    sched_.axis_idx < sched_.axes.size() ? sched_.axes[sched_.axis_idx] : "-";
  add("state", to_string(state_));
  add("axis", axis);
  add(
    "axis_index", sched_.axes.empty() ? "-" :
    std::to_string(sched_.axis_idx + 1) + "/" + std::to_string(sched_.axes.size()));
  add(
    "rung", std::to_string(round_ + 1) + "/" + std::to_string(max_rounds_));
  add("round", std::to_string(round_ + 1) + "/" + std::to_string(max_rounds_));
  add("wn_target", fmt(decision_.wn_target, 2));
  add("precondition", precondition_);
  add(
    "episode",
    std::to_string(sched_.rep + 1) + "/" + std::to_string(sched_.episodes_per_rung));
  // Session-level progress for the ground station: how many test steps of
  // the planned total are behind us, and the phase in plain words. DONE
  // reports 100% even when buckets closed early -- finished is finished.
  {
    const auto [done, total] = session_progress(
      sched_.axes, static_cast<size_t>(max_rounds_), sched_.episodes_per_rung,
      round_, sched_.axis_idx, sched_.rep, false);
    const bool finished = state_ == TunerState::DONE;
    add("steps_done", std::to_string(finished ? total : done));
    add("steps_total", std::to_string(total));
    add(
      "progress_pct",
      std::to_string(
        finished ? 100 : (total > 0 ? (100 * done) / total : 0)));
  }
  add("phase", phase_string(state_));
  add("zeta_target", fmt(zeta_target_, 2));
  add("step_size", fmt(sched_.step_size, 2));
  add("step_size_z", fmt(sched_.step_size_z, 2));
  add("yaw_step", fmt(sched_.yaw_step, 2));
  add("step_max", fmt(sched_.step_ceiling(safety_.limits().max_pos_error), 2));
  add(
    "envelope", "+/-" + fmt(sched_.step_size, 2) + " m lat, +/-" +
    fmt(sched_.step_size_z, 2) + " m vert, +/-" + fmt(sched_.yaw_step, 2) + " rad yaw");
  add("start_requested", start_requested_ ? "true" : "false");
  add("require_enable", require_enable_ ? "true" : "false");
  add("offboard", in_offboard() ? "true" : "false");
  add("px4_mode", px4_mode_.value_or("-"));
  add("abort_reason", abort_reason_);
  add("diagnosis", diagnosis_);
  add("report_path", report_path_);

  if (gains_) {
    for (const auto & [ax, kxkv] : *gains_) {
      const auto [wn, zeta] = wn_zeta_from_pd(kxkv.first, kxkv.second);
      add("gain_" + ax, "wn=" + fmt(wn, 2) + " zeta=" + fmt(zeta, 2));
    }
  }
  if (yaw_tau_) {
    add("yaw_tau", fmt(*yaw_tau_, 3));
  }
  // Old vs new: the session baseline and per-axis outcomes the result view
  // needs. baseline_* stays fixed for the whole session, unlike safe gains.
  if (baseline_gains_) {
    for (const auto & [ax, kxkv] : *baseline_gains_) {
      const auto [wn, zeta] = wn_zeta_from_pd(kxkv.first, kxkv.second);
      add("baseline_" + ax, "wn=" + fmt(wn, 2) + " zeta=" + fmt(zeta, 2));
    }
  }
  if (baseline_yaw_tau_) {
    add("baseline_yaw_tau", fmt(*baseline_yaw_tau_, 3));
  }
  for (const auto & [ax, note] : session_result_) {
    add("result_" + ax, note);
  }
  add(
    "trim", fmt(a_trim_[0], 2) + "," + fmt(a_trim_[1], 2) + "," + fmt(a_trim_[2], 2));

  // Where the vehicle is relative to what the session commands: the
  // numbers a pilot watches while deciding whether to let it continue.
  if (sched_.odom) {
    std::vector<std::string> err;
    for (int i = 0; i < 3; ++i) {
      err.push_back(fmt(sched_.setpoint[i] - sched_.odom->pos[i], 2));
    }
    add("pos_err", join(err, ","));
    add("altitude", fmt(sched_.odom->pos[2], 2));
  }
  // Altitude, as the gate judges it. "altitude" above is the raw odometry
  // z, which is not height above ground -- keep both, and label them.
  const auto agl = agl_now();
  add("agl", agl ? fmt(*agl, 2) : "-");
  add("agl_source", agl_source());
  const auto h_agl = hover_agl();
  add("hover_agl", h_agl ? fmt(*h_agl, 2) : "-");
  add("hover_mode", hover_mode_);
  add(
    "hover_point",
    fmt(hover_pt_[0], 2) + "," + fmt(hover_pt_[1], 2) + "," + fmt(hover_pt_[2], 2));
  add("hover_yaw", fmt(hover_yaw_, 2));
  add("min_tuning_altitude", fmt(min_tuning_altitude_, 2));
  add("min_start_altitude", fmt(min_start_altitude(), 2));
  add("min_altitude", fmt(safety_.limits().min_altitude, 2));
  add("z_step_clearance", fmt(z_step_clearance(), 2));
  {
    const auto [ok, why] = altitude_gate();
    add("altitude_gate", ok ? "ok" : why);
  }
  add(
    "setpoint",
    fmt(sched_.setpoint[0], 2) + "," + fmt(sched_.setpoint[1], 2) + "," +
    fmt(sched_.setpoint[2], 2));
  add("results", std::to_string(results_.size()));
  add("restore", restore_state_.empty() ? "-" : restore_state_);

  health_pub_->publish(st);
}

void TuningConductor::abort(const std::string & reason)
{
  if (state_ == TunerState::ABORT) {return;}
  abort_reason_ = reason;
  RCLCPP_ERROR(get_logger(), "ABORT: %s", reason.c_str());
  // Only meaningful while the vehicle was actually tracking a setpoint it
  // had had time to reach. During an approach the error is the approach.
  if (diagnosis_.empty() && sched_.odom && state_ != TunerState::GOTO_HOVER &&
    state_ != TunerState::TOO_LOW &&
    std::abs(sched_.setpoint[2] - sched_.odom->pos[2]) > 0.5)
  {
    diagnosis_ =
      "large altitude error at abort: likely thrust-map error. Verify "
      "max_thrust in geometric_mavros.yaml - fly a Position-mode hover and run "
      "geo-tuner-hover on the ulog.";
  }
  // Hold where the vehicle IS. An abort that commands a point the vehicle
  // is not at is a manoeuvre, and the bigger the trouble the bigger the
  // manoeuvre: in the first field session the abort re-commanded a hover
  // point 13.8 m above the vehicle and flew it there at full thrust.
  hold_here();
  // Unvalidated changes do not survive an abort: position gains AND
  // yawctrl_tau go back to the last validated (or entry) values. The FULL
  // safe set is always sent, not only when the books say something differs:
  // the books record intent, and an earlier write may have been lost (a
  // validation-failure restore that timed out left failed gains live while
  // gains_ already equalled safe_gains_). st_abort confirms the answer.
  restore_tries_ = 0;
  if (safe_gains_ || (yaw_tau_ && safe_yaw_tau_)) {
    if (safe_gains_) {gains_ = safe_gains_;}
    if (yaw_tau_ && safe_yaw_tau_) {yaw_tau_ = safe_yaw_tau_;}
    send_restore(now_s());
  } else {
    restore_state_.clear();   // no gains were read: nothing was ever changed
  }
  freeze_session_end();
  save_session_recording();
  write_report("aborted: " + reason);
  goto_state(TunerState::ABORT);
}

void TuningConductor::send_restore(double now)
{
  ++restore_tries_;
  restore_t0_ = now;
  restore_state_ = "pending";
  apply_gains(safe_gains_ ? *safe_gains_ : Gains{},
    yaw_tau_ && safe_yaw_tau_ ? safe_yaw_tau_ : std::nullopt);
  RCLCPP_WARN(get_logger(), "Restoring last known-safe gains (attempt %d)", restore_tries_);
}

void TuningConductor::st_abort(double now)
{
  if (restore_state_ != "pending") {return;}
  constexpr int kMaxTries = 5;
  std::string failure;
  if (pending_set_ && ready(*pending_set_)) {
    const auto res = pending_set_->get();
    pending_set_.reset();
    bool ok = res != nullptr;
    if (res) {
      for (const auto & r : res->results) {ok = ok && r.successful;}
    }
    if (ok) {
      restore_state_ = "confirmed";
      std::string text = "gain restore confirmed by the controller:";
      if (safe_gains_) {
        for (const auto & [ax, kxkv] : *safe_gains_) {
          text += " " + ax + " " + fmt(kxkv.first, 3) + "/" + fmt(kxkv.second, 3);
        }
      }
      if (yaw_tau_ && safe_yaw_tau_) {text += ", yawctrl_tau " + fmt(*safe_yaw_tau_, 3);}
      status(text);
      return;
    }
    failure = "controller rejected the restore";
  } else if (now - restore_t0_ > service_timeout_) {
    pending_set_.reset();
    failure = "no answer to the restore in " + fmt(service_timeout_, 0) + " s";
  } else {
    return;
  }
  if (restore_tries_ < kMaxTries) {
    status(failure + "; retrying (" + std::to_string(restore_tries_ + 1) + "/" +
      std::to_string(kMaxTries) + ")");
    send_restore(now);
    return;
  }
  restore_state_ = "UNCONFIRMED";
  RCLCPP_ERROR(get_logger(), "GAIN RESTORE UNCONFIRMED after %d attempts: %s", kMaxTries,
    failure.c_str());
  status("GAIN RESTORE UNCONFIRMED (" + failure + "): the controller may still run unvalidated "
    "gains. Land, then check gains.pos/vel and yawctrl_tau with ros2 param get.");
}

// ---------------------------------------------------------------------

void TuningConductor::freeze_session_end()
{
  session_end_ = SessionEnd{now_s(), a_trim_, local_stamp()};
}

void TuningConductor::write_report(const std::string & status_text)
{
  // Final gains with what the session concluded about them. No "effective"
  // wn/zeta computed from the alpha that designed them: that restates the
  // design target and proves nothing. The evidence is the identification
  // (with its interval), the phase margins, and the validation result.
  YAML::Node final_gains(YAML::NodeType::Map);
  if (gains_) {
    for (const auto & [ax, kxkv] : *gains_) {
      const double kx = kxkv.first, kv = kxkv.second;
      const auto [wn, zeta] = wn_zeta_from_pd(kx, kv);
      YAML::Node f;
      f["kx"] = yaml_double(round_to(kx, 3));
      f["kv"] = yaml_double(round_to(kv, 3));
      f["wn_nominal"] = yaml_double(round_to(wn, 3));
      f["zeta_nominal"] = yaml_double(round_to(zeta, 3));
      const auto it = axis_prog_.find(ax);
      if (it != axis_prog_.end()) {
        const auto & id = it->second.id_last;
        f["result"] = it->second.result;
        f["outcome"] = it->second.outcome;
        f["validated"] = it->second.validated;
        if (id.ok) {
          f["alpha"] = yaml_double(round_to(id.alpha, 3));
          f["alpha_ci"] = std::vector<double>{round_to(id.alpha_lo, 3), round_to(id.alpha_hi, 3)};
          f["lag_ms"] = yaml_double(round_to(1e3 * id.lag, 0));
          f["lag_ci_ms"] = std::vector<double>{round_to(1e3 * id.lag_lo, 0),
            round_to(1e3 * id.lag_hi, 0)};
          f["pm_nominal_deg"] = yaml_double(round_to(nominal_phase_margin_deg(kx, kv, id), 1));
          f["pm_worst_deg"] = yaml_double(round_to(worst_phase_margin_deg(kx, kv, id), 1));
        }
      }
      final_gains[ax] = f;
    }
  }

  YAML::Node episodes(YAML::NodeType::Sequence);
  for (const auto & r : results_) {episodes.push_back(r);}


  // Session facts come from its end, not from the moment the report is
  // written: accept usually happens on the ground (see session_end_).
  YAML::Node trim(YAML::NodeType::Sequence);
  for (double a : session_end_ ? session_end_->trim : a_trim_) {
    trim.push_back(yaml_double(round_to(a, 3)));
  }
  const std::string written_at = local_stamp();

  YAML::Node snippet_gains(YAML::NodeType::Map);
  if (final_gains.size() > 0) {
    YAML::Node pos(YAML::NodeType::Map), vel(YAML::NodeType::Map);
    for (const auto & kv : final_gains) {
      const auto ax = kv.first.as<std::string>();
      // By value, not by node: assigning the node itself makes the two
      // trees share it, and the emitter then writes a YAML anchor/alias
      // (&1 / *1) where the report is supposed to carry a plain number.
      pos[ax] = yaml_double(kv.second["kx"].as<double>());
      vel[ax] = yaml_double(kv.second["kv"].as<double>());
    }
    snippet_gains["pos"] = pos;
    snippet_gains["vel"] = vel;
  }
  YAML::Node snippet;
  snippet["gains"] = snippet_gains;

  YAML::Node report;
  report["status"] = status_text;
  report["diagnosis"] = diagnosis_;
  report["accel_trim"] = trim;
  if (yaw_tau_) {
    report["final_yawctrl_tau"] = yaml_double(round_to(*yaw_tau_, 3));
    const auto yit = axis_prog_.find("yaw");
    if (yit != axis_prog_.end()) {report["yaw_result"] = yit->second.result;}
  } else {
    report["final_yawctrl_tau"] = YAML::Node(YAML::NodeType::Null);
  }
  report["time"] = session_end_ ? session_end_->stamp : written_at;
  if (status_text == "accepted") {report["accepted_at"] = written_at;}
  if (session_t0_ >= 0.0) {
    const double t_end = session_end_ ? session_end_->t : now_s();
    report["session_duration_s"] = yaml_double(round_to(t_end - session_t0_, 1));
  }
  report["zeta_target"] = yaml_double(zeta_target_);
  report["wn_target"] = yaml_double(decision_.wn_target);
  report["wn_target_z"] = yaml_double(wn_target_z_);
  report["pm_nominal_deg"] = yaml_double(decision_.margins.nominal_deg);
  report["pm_worst_deg"] = yaml_double(decision_.margins.worst_deg);
  report["rounds_flown"] = static_cast<int>(round_ + 1);
  report["session_recording"] = session_csv_;
  report["episodes"] = episodes;
  report["final_gains"] = final_gains;
  report["controller_yaml_snippet"] = snippet;

  std::error_code ec;   // best effort; the open below reports real failures
  const auto parent = std::filesystem::path(report_path_).parent_path();
  if (!parent.empty()) {std::filesystem::create_directories(parent, ec);}
  std::ofstream f(report_path_);
  if (!f) {
    RCLCPP_ERROR(get_logger(), "could not write report: %s", report_path_.c_str());
    return;
  }
  YAML::Emitter out;
  out << report;
  f << out.c_str() << "\n";
}

}  // namespace geo_tuner
