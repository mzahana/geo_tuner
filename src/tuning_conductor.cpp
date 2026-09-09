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
         s == TunerState::UPDATE_GAINS;
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
  z_step_margin_ = declare_parameter<double>("z_step_margin", 0.5);
  // AGL source. Odometry z is NOT height above ground: PX4's local frame
  // origin sat 6.6 m below the ground in the first field session, which
  // put every altitude limit 6.6 m out. rel_alt is height above the home
  // point; ground_z is the fallback offset when no such topic exists.
  const auto agl_topic =
    declare_parameter<std::string>("agl_topic", "mavros/global_position/rel_alt");
  agl_timeout_ = declare_parameter<double>("agl_timeout", 2.0);            // s
  ground_z_ = declare_parameter<double>("ground_z", 0.0);                  // m, odom frame
  sched_.step_size = declare_parameter<double>("step_size", 0.5);        // m
  sched_.step_size_z = declare_parameter<double>("step_size_z", 0.4);    // m
  settle_time_ = declare_parameter<double>("settle_time", 4.0);          // s before each step
  hover_timeout_ = declare_parameter<double>("hover_timeout", 20.0);     // s to reach hover point
  episode_time_ = declare_parameter<double>("episode_time", 6.0);        // s of recording
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
  // Yaw is a time-constant target, not a wn target: it does not ladder, so
  // a yaw bucket per rung re-identifies the same thing. Fly it once, on
  // the final rung (the 2026-09-09 field session spent 6 of 24 episodes
  // on yaw for at most one tau update's worth of information).
  yaw_final_rung_only_ = declare_parameter<bool>("yaw_final_rung_only", true);
  // Per-episode quality screen for the yaw fit. The closed-loop (position)
  // fit keeps its 0.15: yaw is looser because a 0.5 rad heading response
  // carries more relative sensor noise, and the real gate on what reaches
  // yawctrl_tau is the bucket median + estimate_consistency. On 2026-09-09
  // the fixed 0.15 rejected 5 of 6 yaw episodes at nrmse 0.148-0.172 whose
  // T estimates agreed to ~1.3x -- consistent evidence thrown away.
  yaw_fit_nrmse_ = declare_parameter<double>("yaw_fit_nrmse", 0.25);
  // wn ladder: identify at each rung before pushing bandwidth up
  wn_ladder_ = declare_parameter<std::vector<double>>("wn_ladder", {1.2, 1.6, 2.0});
  zeta_target_ = declare_parameter<double>("zeta_target", 0.95);
  max_change_ = declare_parameter<double>("max_gain_change_factor", 1.6);
  // How far the identified loop must stay clear of the Routh-Hurwitz
  // stability boundary before the ladder is allowed to climb. 1.0 is the
  // boundary itself; the default keeps a 4x margin on the Routh product.
  stability_margin_ = declare_parameter<double>("stability_margin", 4.0);
  // Repeat each step N times per (axis, rung) and update from the MEDIAN
  // identified alpha/T: single-episode fits are noisy (the lateral
  // response is truly higher-order) and that noise maps 1:1 into the
  // gains. The consistency gate refuses any update when the accepted
  // estimates disagree by more than this factor.
  sched_.episodes_per_rung =
    std::max(1, static_cast<int>(declare_parameter<int>("episodes_per_rung", 3)));
  consistency_ = declare_parameter<double>("estimate_consistency", 1.35);
  // --- session-time reduction (see docs/TUNING_GUIDE.md) ---
  // Sequential stopping: fly at least this many reps, and stop the bucket
  // early when every flown episode was accepted AND they agree within
  // early_stop_spread -- a gate STRICTER than estimate_consistency, so an
  // early stop only happens on evidence better than what a full bucket is
  // required to produce.
  sched_.min_episodes = std::max(
    1, std::min(
      static_cast<int>(declare_parameter<int>("min_episodes_per_rung", 2)),
      sched_.episodes_per_rung));
  sched_.early_stop_spread = declare_parameter<double>("early_stop_spread", 1.15);
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
  sched_.settle_tol_pos = declare_parameter<double>("settle_tol_pos", 0.06);   // m
  sched_.settle_tol_vel = declare_parameter<double>("settle_tol_vel", 0.10);   // m/s
  sched_.settle_tol_yaw = declare_parameter<double>("settle_tol_yaw", 0.05);   // rad
  // Adaptive episode length: stop recording once the response has been
  // flat at its steady state for episode_quiet_time, but never before the
  // transient can have finished (min_episode_time and
  // episode_settle_periods/(zeta*wn)). episode_time is the cap.
  adaptive_episode_ = declare_parameter<bool>("adaptive_episode", true);
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
  // indexes axes[axis_idx] and wn_ladder[rung] unguarded, and a vehicle in
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
  if (wn_ladder_.empty()) {
    RCLCPP_ERROR(get_logger(), "wn_ladder is empty; using [1.2, 1.6, 2.0]");
    wn_ladder_ = {1.2, 1.6, 2.0};
  }

  // ---- interfaces ----
  sp_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    setpoint_topic, 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("geo_tuner/status", 10);
  // Structured status for a ground station. The String above is a human
  // log line that only appears when something happens; a panel needs the
  // state, the progress through the ladder, and the reason it is waiting,
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
  std::vector<std::string> ladder;
  for (double w : wn_ladder_) {ladder.push_back(fmt(w, 2));}
  RCLCPP_INFO(
    get_logger(), "Tuning conductor up. axes=[%s] wn_ladder=[%s] zeta=%s hover=%s",
    join(sched_.axes, ", ").c_str(), join(ladder, ", ").c_str(),
    fmt(zeta_target_, 2).c_str(), hov.str().c_str());
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
  if (session_t0_ < 0.0) {session_t0_ = now_s();}
  // The schedule may be parked on a bucket this session skips (a yaw-only
  // axes list with yaw_final_rung_only, say): move to one it flies.
  if (!seek_eligible_axis()) {return;}
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
    "yawctrl_tau", "attctrl_tau"};
  auto fut = get_param_cli_->async_send_request(req);
  get_req_id_ = fut.request_id;
  pending_get_ = fut.future.share();
  request_t0_ = now ? *now : now_s();
  ++request_tries_;
}

void TuningConductor::apply_yaw_tau(double tau)
{
  auto req = std::make_shared<SetParameters::Request>();
  rcl_interfaces::msg::Parameter p;
  p.name = "yawctrl_tau";
  p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  p.value.double_value = tau;
  req->parameters.push_back(p);
  pending_set_ = set_param_cli_->async_send_request(req).future.share();
}

void TuningConductor::apply_gains(const Gains & gains)
{
  auto req = std::make_shared<SetParameters::Request>();
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
    }
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

  switch (state_) {
    case TunerState::WAIT_ODOM: st_wait_odom(now); break;
    case TunerState::WAIT_ENABLE: st_wait_enable(now); break;
    case TunerState::WAIT_OFFBOARD: st_wait_offboard(now); break;
    case TunerState::TOO_LOW: st_too_low(now); break;
    case TunerState::GOTO_HOVER: st_goto_hover(now); break;
    case TunerState::SETTLE: st_settle(now); break;
    case TunerState::STEP: st_step(now); break;
    case TunerState::ANALYZE: st_analyze(now); break;
    case TunerState::UPDATE_GAINS: st_update_gains(now); break;
    case TunerState::DONE: break;    // keep publishing hover setpoint
    case TunerState::ABORT: break;   // hold hover; pilot takes over via RC
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
    std::vector<double> vals;
    for (size_t i = 0; i < 6; ++i) {vals.push_back(res->values[i].double_value);}
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
      // Seeds the in-loop lag prior for the closed-loop identification.
      attctrl_tau_ = att_tau;
    }
    if (yaw_tau > 0.0) {
      yaw_tau_ = yaw_tau;
    } else if (att_tau > 0.0) {
      yaw_tau_ = att_tau;
    }
    baseline_yaw_tau_ = yaw_tau_;
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
    baseline_read_ = true;
    if (require_enable_ && !start_requested_) {
      status("Gains read. Waiting for the ~/start service (require_enable is set)");
      return;
    }
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
      offboard_mode_ + " engaged; resuming (rung " + std::to_string(rung_ + 1) + "/" +
      std::to_string(wn_ladder_.size()) + ", axis " + sched_.axes[sched_.axis_idx] + ")");
    begin_flying();
  }
}

void TuningConductor::st_too_low(double now)
{
  // Holding where the pilot handed over, because that was below the
  // working altitude. Nothing is commanded to move: the way out is the
  // pilot climbing (which needs manual control, so the mode change back
  // to OFFBOARD re-enters here) or the operator lowering the gate.
  const auto [ok, why] = altitude_gate();
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
      "At hover point; settling (<= " + fmt(settle_time_, 1) + "s) (rung " +
      std::to_string(rung_ + 1) + "/" + std::to_string(wn_ladder_.size()) +
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

void TuningConductor::st_settle(double now)
{
  const double elapsed = now - state_t0_;
  if (elapsed < min_settle_time_) {return;}
  // settle_time remains the hard cap: with a noisy/windy plant this
  // degrades exactly to the previous fixed-time behaviour.
  if (elapsed < settle_time_ && !sched_.is_quiet(now)) {return;}
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
  if (adaptive_episode_ && elapsed >= episode_min_time_ && sched_.odom &&
    sched_.response_settled(sched_.odom->t_stamp))
  {
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
      "_" << ax << "_rung" << rung_ << "_rep" << sched_.rep << ".csv";
    std::ofstream f(std::filesystem::path(dir) / name.str());
    f << "# axis=" << ax << " step=" << step << " rung=" << rung_ <<
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
  rec["rung"] = static_cast<int>(rung_);
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
    sched_.bucket.add(fit.T, fit.delay);
  }
  results_.push_back(rec);
  episode_finished();
}

void TuningConductor::finalize_yaw_bucket()
{
  // All reps for this yaw bucket flown: aggregate the accepted T
  // estimates and update yawctrl_tau from their median.
  const auto est = robust_ratio_estimate(
    sched_.bucket.alphas, std::min(2, sched_.episodes_per_rung), consistency_);
  YAML::Node rec;
  rec["axis"] = "yaw";
  rec["rung"] = static_cast<int>(rung_);
  rec["n_episodes"] = sched_.rep;
  rec["n_used"] = est.n_used;
  if (std::isfinite(est.spread)) {
    rec["spread"] = yaml_double(round_to(est.spread, 3));
  } else {
    rec["spread"] = YAML::Node(YAML::NodeType::Null);
  }
  if (!est.ok) {
    rec["action"] = "keeping yawctrl_tau (" + est.reason + ")";
    results_.push_back(rec);
    session_result_["yaw"] = "kept: " + est.reason;
    status("yaw: " + est.reason + "; keeping tau");
    advance_axis();
    return;
  }
  const double T_med = est.value;
  // T scales with the applied tau through the same (unknown) efficiency
  // factor, which cancels in the ratio update.
  double tau_new = *yaw_tau_ * yaw_T_target_ / T_med;
  tau_new = std::clamp(tau_new, *yaw_tau_ / max_change_, *yaw_tau_ * max_change_);
  tau_new = std::clamp(tau_new, yaw_tau_min_, yaw_tau_max_);
  rec["T_median"] = yaml_double(round_to(T_med, 3));
  rec["yaw_tau_new"] = yaml_double(round_to(tau_new, 3));
  rec["action"] = "tau updated (median)";
  results_.push_back(rec);
  session_result_["yaw"] =
    "T " + fmt(T_med, 2) + "s (n=" + std::to_string(est.n_used) + ", spread " +
    fmt(est.spread, 2) + "x)";
  status(
    "yaw: median T=" + fmt(T_med, 2) + "s (n=" + std::to_string(est.n_used) +
    ", spread " + fmt(est.spread, 2) + "x) -> tau " + fmt(*yaw_tau_, 3) + " -> " +
    fmt(tau_new, 3));
  yaw_tau_ = tau_new;
  apply_yaw_tau(tau_new);
  goto_state(TunerState::UPDATE_GAINS);
}

void TuningConductor::st_analyze(double)
{
  const std::string ax = sched_.axes[sched_.axis_idx];
  if (ax == "yaw") {
    analyze_yaw();
    return;
  }
  const double step = sched_.step_applied;
  Eigen::VectorXd t(sched_.recording.size()), y(sched_.recording.size());
  for (size_t i = 0; i < sched_.recording.size(); ++i) {
    t[static_cast<Eigen::Index>(i)] = sched_.recording[i].first;
    y[static_cast<Eigen::Index>(i)] = sched_.recording[i].second;
  }
  dump_episode(ax, t, y, step);
  const double kx_now = gains_->at(ax).first;
  const double kv_now = gains_->at(ax).second;
  const double wn_target = wn_ladder_[rung_];
  YAML::Node rec;
  rec["axis"] = ax;
  rec["rung"] = static_cast<int>(rung_);
  rec["rep"] = sched_.rep;
  rec["wn_target"] = yaml_double(wn_target);
  rec["step"] = yaml_double(step);
  rec["kx_applied"] = yaml_double(round_to(kx_now, 3));
  rec["kv_applied"] = yaml_double(round_to(kv_now, 3));
  rec["settle_s"] = yaml_double(round_to(last_settle_dur_, 2));
  rec["record_s"] = yaml_double(round_to(t.size() > 0 ? t[t.size() - 1] : 0.0, 2));

  LoopFitResult fit;
  try {
    // Identify against the loop we actually commanded: kx and kv are
    // known, so the only dynamic unknowns are the plant-gain factor and
    // the in-loop lag. attctrl_tau seeds the lag; it does not constrain it.
    fit = fit_closed_loop(t, y, step, kx_now, kv_now, attctrl_tau_.value_or(0.15));
    // tau carries the transport delay too; see loop_fit.hpp.
  } catch (const std::exception & e) {
    rec["action"] = std::string("episode discarded (fit error: ") + e.what() + ")";
    results_.push_back(rec);
    status("closed-loop fit failed on " + ax + " (" + e.what() + "); discarding episode");
    episode_finished();
    return;
  }

  const double wn_meas = fit.wn_effective(kx_now);
  rec["alpha"] = yaml_double(round_to(fit.alpha, 3));
  rec["tau_lag"] = yaml_double(round_to(fit.tau, 3));
  rec["nrmse"] = yaml_double(round_to(fit.nrmse, 3));
  rec["overshoot"] = yaml_double(round_to(fit.overshoot, 3));
  rec["wn_meas"] = yaml_double(round_to(wn_meas, 3));
  rec["zeta_meas"] = yaml_double(round_to(fit.zeta_effective(kx_now, kv_now), 3));
  status(
    ax + ": alpha=" + fmt(fit.alpha, 2) + " tau=" + fmt(fit.tau * 1e3, 0) +
    "ms nrmse=" + fmt(fit.nrmse, 3) +
    " (wn_eff=" + fmt(wn_meas, 2) + ") os=" + fmt(fit.overshoot * 100.0, 0) + "%");

  if (!fit.ok()) {
    rec["action"] = fit.ambiguous ?
      "fit ambiguous (near-equal minima disagree on alpha)" :
      (fit.at_bounds ? "fit rested on a solver bound" : "fit rejected");
    results_.push_back(rec);
    status("Fit quality gate failed on " + ax + " (" +
      rec["action"].as<std::string>() + "); episode discarded");
    episode_finished();
    return;
  }

  // Plausibility gate. Unlike the old fit, alpha here was estimated
  // WITHOUT being confined to this range, so a value outside it is real
  // evidence that the episode was corrupted rather than an artefact of
  // the estimator being held inside a prior.
  if (!(fit.alpha >= kAlphaMin && fit.alpha <= kAlphaMax)) {
    rec["action"] = "alpha implausible; episode discarded";
    results_.push_back(rec);
    status(
      ax + ": alpha=" + fmt(fit.alpha, 2) + " outside [" + fmt(kAlphaMin, 1) + ", " +
      fmt(kAlphaMax, 1) + "]; discarding episode");
    episode_finished();
    return;
  }

  rec["action"] = "accepted";
  results_.push_back(rec);
  sched_.bucket.add(fit.alpha, fit.tau);
  episode_finished();
}

void TuningConductor::finalize_pos_bucket(const std::string & ax)
{
  // All reps for this (axis, rung) flown: aggregate the accepted alpha
  // estimates and update gains from their median.
  const double kx_now = gains_->at(ax).first;
  const double kv_now = gains_->at(ax).second;
  const double wn_target = wn_ladder_[rung_];
  const auto est = robust_ratio_estimate(
    sched_.bucket.alphas, std::min(2, sched_.episodes_per_rung), consistency_);
  YAML::Node rec;
  rec["axis"] = ax;
  rec["rung"] = static_cast<int>(rung_);
  rec["wn_target"] = yaml_double(wn_target);
  rec["n_episodes"] = sched_.rep;
  rec["n_used"] = est.n_used;
  if (std::isfinite(est.spread)) {
    rec["spread"] = yaml_double(round_to(est.spread, 3));
  } else {
    rec["spread"] = YAML::Node(YAML::NodeType::Null);
  }
  if (!est.ok) {
    rec["action"] = "keeping gains (" + est.reason + ")";
    results_.push_back(rec);
    session_result_[ax] = "kept: " + est.reason;
    status(ax + ": " + est.reason + "; keeping gains");
    advance_axis();
    return;
  }

  // Stability margin, from the identified in-loop lag rather than a rule
  // of thumb. The closed loop is tau*s^3 + s^2 + alpha*kv*s + alpha*kx,
  // and Routh-Hurwitz puts the instability boundary at kv = tau*kx. With
  // the design rule kx = wn^2, kv = 2*zeta*wn that is wn = 2*zeta/tau, so
  // holding the Routh product `stability_margin` times clear of the
  // boundary caps the ladder at
  //
  //     wn_target <= 2*zeta_target / (stability_margin * tau)
  //
  // At the default margin of 4 and zeta 0.95 this is wn*tau <= 0.475,
  // which is deliberately about as conservative as the wn*delay <= 0.45
  // heuristic it replaces -- but derived from a measured quantity, and it
  // now scales correctly when zeta_target is changed.
  const double tau_med = sched_.bucket.median_lag();
  const double wn_cap = tau_med > 0.0 ?
    2.0 * zeta_target_ / (stability_margin_ * tau_med) :
    std::numeric_limits<double>::infinity();
  rec["tau_median"] = yaml_double(round_to(tau_med, 3));
  rec["wn_stability_cap"] = yaml_double(round_to(wn_cap, 3));
  if (wn_target > wn_cap) {
    const std::string action =
      "wn_target " + fmt(wn_target, 2) + " exceeds the stability cap " +
      fmt(wn_cap, 2) + " rad/s implied by the measured in-loop lag " +
      fmt(tau_med * 1e3, 0) + " ms (margin " + fmt(stability_margin_, 1) +
      "x on the Routh product); ladder stopped";
    rec["action"] = action;
    results_.push_back(rec);
    session_result_[ax] = "kept: stability cap " + fmt(wn_cap, 2) + " rad/s (lag " +
      fmt(tau_med * 1e3, 0) + " ms)";
    status(action);
    finish();
    return;
  }

  const double alpha = est.value;
  auto corr = correct_gains_from_identification(
    kx_now, std::sqrt(alpha * kx_now), wn_target, zeta_target_);
  // Rate-limit gain changes per bucket
  const double kx_new =
    std::clamp(corr.kx_new, kx_now / max_change_, kx_now * max_change_);
  const double kv_new =
    std::clamp(corr.kv_new, kv_now / max_change_, kv_now * max_change_);
  rec["alpha"] = yaml_double(round_to(alpha, 3));
  rec["kx_new"] = yaml_double(round_to(kx_new, 3));
  rec["kv_new"] = yaml_double(round_to(kv_new, 3));
  rec["action"] = "gains updated (median)";
  results_.push_back(rec);

  safe_gains_ = gains_;   // current set flew safely
  session_result_[ax] =
    "alpha " + fmt(alpha, 2) + " (n=" + std::to_string(est.n_used) + ", spread " +
    fmt(est.spread, 2) + "x)";
  (*gains_)[ax] = {kx_new, kv_new};
  apply_gains(Gains{{ax, gains_->at(ax)}});
  status(
    ax + ": median alpha=" + fmt(alpha, 2) + " (n=" + std::to_string(est.n_used) +
    ", spread " + fmt(est.spread, 2) + "x) -> kx " + fmt(kx_now, 2) + "->" +
    fmt(kx_new, 2) + ", kv " + fmt(kv_now, 2) + "->" + fmt(kv_new, 2));
  goto_state(TunerState::UPDATE_GAINS);
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
  advance_axis();
}

void TuningConductor::episode_finished()
{
  // One step episode analyzed (accepted or discarded). Fly the next
  // repetition of the same (axis, rung), or -- when the bucket is full
  // (or already consistent enough) -- aggregate and decide on a gain
  // update.
  //
  // Alternate step direction to stay centered on the hover point.
  // Bidirectional: flip only after the leg that returns to centre, giving
  // the 0 -> +d -> 0 -> -d -> 0 sequence.
  if (!sched_.bidirectional || sched_.leg_offset == 0.0) {
    sched_.step_sign *= -1.0;
  }
  ++sched_.rep;
  if (sched_.rep < sched_.episodes_per_rung && !sched_.bucket_settled()) {
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
    return;
  }
  if (sched_.bucket.count() == sched_.rep && sched_.rep < sched_.episodes_per_rung) {
    status(
      "bucket consistent after " + std::to_string(sched_.rep) + " episodes (spread <= " +
      fmt(sched_.early_stop_spread, 2) + "x); stopping early");
  }
  const std::string ax = sched_.axes[sched_.axis_idx];
  if (ax == "yaw") {
    finalize_yaw_bucket();
  } else {
    finalize_pos_bucket(ax);
  }
}

void TuningConductor::advance_axis()
{
  if (sched_.advance_axis(hover_pt_, hover_yaw_)) {
    ++rung_;
    if (rung_ >= wn_ladder_.size()) {
      finish();
      return;
    }
  }
  if (!seek_eligible_axis()) {return;}
  goto_state(TunerState::GOTO_HOVER);
}

bool TuningConductor::seek_eligible_axis()
{
  while (!axis_eligible(
      sched_.axes[sched_.axis_idx], rung_, wn_ladder_.size(), yaw_final_rung_only_))
  {
    status(
      "skipping " + sched_.axes[sched_.axis_idx] + " on rung " +
      std::to_string(rung_ + 1) + " (tuned once, on the final rung)");
    if (sched_.advance_axis(hover_pt_, hover_yaw_)) {
      ++rung_;
      if (rung_ >= wn_ladder_.size()) {
        finish();
        return false;
      }
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
  res->message = "Aborted: gains restored, holding hover.";
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

  // Fly on whatever gains are currently in force: an abort has already
  // restored the safe set, and silently changing them here would hide
  // what the vehicle is actually using.
  abort_reason_.clear();
  diagnosis_.clear();
  results_.clear();
  sched_.recording.clear();
  sched_.bucket = EpisodeBucket();
  rung_ = 0;
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
    "rung", wn_ladder_.empty() ? "-" :
    std::to_string(rung_ + 1) + "/" + std::to_string(wn_ladder_.size()));
  add("wn_target", rung_ < wn_ladder_.size() ? fmt(wn_ladder_[rung_], 2) : "-");
  add(
    "episode",
    std::to_string(sched_.rep + 1) + "/" + std::to_string(sched_.episodes_per_rung));
  // Session-level progress for the ground station: how many test steps of
  // the planned total are behind us, and the phase in plain words. DONE
  // reports 100% even when buckets closed early -- finished is finished.
  {
    const auto [done, total] = session_progress(
      sched_.axes, wn_ladder_.size(), sched_.episodes_per_rung,
      rung_, sched_.axis_idx, sched_.rep, yaw_final_rung_only_);
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
  if (safe_gains_ && gains_ != safe_gains_) {
    gains_ = safe_gains_;
    apply_gains(*gains_);
    RCLCPP_WARN(get_logger(), "Restored last known-safe gains");
  }
  write_report("aborted: " + reason);
  goto_state(TunerState::ABORT);
}

// ---------------------------------------------------------------------

void TuningConductor::write_report(const std::string & status_text)
{
  // last identified plant-gain factor per axis (lumps thrust-map error
  // and inner-loop lag); effective wn = sqrt(alpha * kx)
  std::map<std::string, double> alpha_by_axis;
  for (const auto & r : results_) {
    if (r["alpha"]) {
      alpha_by_axis[r["axis"].as<std::string>()] = r["alpha"].as<double>();
    }
  }
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
      const auto it = alpha_by_axis.find(ax);
      if (it != alpha_by_axis.end()) {
        const double a = it->second;
        f["alpha"] = yaml_double(a);
        f["wn_effective"] = yaml_double(round_to(std::sqrt(a * kx), 3));
        f["zeta_effective"] = yaml_double(round_to(a * kv / (2.0 * std::sqrt(a * kx)), 3));
      }
      final_gains[ax] = f;
    }
  }

  YAML::Node episodes(YAML::NodeType::Sequence);
  for (const auto & r : results_) {episodes.push_back(r);}

  YAML::Node ladder(YAML::NodeType::Sequence);
  for (double w : wn_ladder_) {ladder.push_back(yaml_double(w));}

  YAML::Node trim(YAML::NodeType::Sequence);
  for (double a : a_trim_) {trim.push_back(yaml_double(round_to(a, 3)));}

  char timebuf[32];
  const std::time_t now_t = std::time(nullptr);
  std::tm tm_local{};
  localtime_r(&now_t, &tm_local);
  std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &tm_local);

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
  } else {
    report["final_yawctrl_tau"] = YAML::Node(YAML::NodeType::Null);
  }
  report["time"] = std::string(timebuf);
  if (session_t0_ >= 0.0) {
    report["session_duration_s"] = yaml_double(round_to(now_s() - session_t0_, 1));
  }
  report["zeta_target"] = yaml_double(zeta_target_);
  report["wn_ladder"] = ladder;
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
