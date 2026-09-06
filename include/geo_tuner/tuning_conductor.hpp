// In-flight auto-tuner for the mav_controllers_ros geometric controller.
//
// Episode-based identification tuner ("Tier 2"): while the vehicle hovers
// in OFFBOARD under the geometric controller, this node
//
//   1. publishes hover setpoints (MultiDOFJointTrajectory, the
//      controller's standard setpoint input -- no custom messages needed);
//   2. injects a small position step on one axis and records the response;
//   3. fits a second-order-plus-delay model to the response;
//   4. corrects kx/kv by pole placement toward the target (wn, zeta) via a
//      live parameter update on the controller node (no landing/restart);
//   5. repeats per axis, walking wn up a conservative ladder, and finally
//      writes a tuned geometric_controller.yaml + full session report.
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

#include <yaml-cpp/yaml.h>

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
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
#include <mavros_msgs/msg/state.hpp>
#endif

#include "geo_tuner/core/safety.hpp"
#include "geo_tuner/tuning_schedule.hpp"

namespace geo_tuner
{

enum class TunerState
{
  WAIT_ODOM,
  WAIT_ENABLE,
  WAIT_OFFBOARD,
  GOTO_HOVER,
  SETTLE,
  STEP,
  ANALYZE,
  UPDATE_GAINS,
  DONE,
  ABORT,
};

const char * to_string(TunerState s);

/// States in which the conductor is actively flying the vehicle (safety
/// monitoring + offboard supervision apply).
bool is_active_state(TunerState s);

// Identified plant-gain factors outside this range are physically
// implausible (thrust maps are not off by >2.5x on a flying vehicle) and
// indicate a corrupted episode -- such identifications are discarded.
inline constexpr double kAlphaMin = 0.4;
inline constexpr double kAlphaMax = 2.5;

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
  void apply_gains(const Gains & gains);

  // ---- state machine ----
  void tick();
  void st_wait_odom(double now);
  void st_wait_enable(double now);
  void st_wait_offboard(double now);
  void st_goto_hover(double now);
  void st_settle(double now);
  void st_step(double now);
  void st_analyze(double now);
  void st_update_gains(double now);

  void analyze_yaw();
  void finalize_yaw_bucket();
  void finalize_pos_bucket(const std::string & ax);
  void episode_finished();
  void advance_axis();
  void finish();
  void set_trim_diagnosis();
  void abort(const std::string & reason);

  // ---- ground-station control ----
  void srv_start(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_abort(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_accept(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_restore_safe(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);
  void srv_reset(Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res);

  void publish_health();
  void write_report(const std::string & status);

  // ---- interfaces ----
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr sp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr health_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
#ifdef GEO_TUNER_HAVE_MAVROS_MSGS
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
#endif
  rclcpp::Client<GetParameters>::SharedPtr get_param_cli_;
  rclcpp::Client<SetParameters>::SharedPtr set_param_cli_;
  rclcpp::Service<Trigger>::SharedPtr start_srv_, abort_srv_, accept_srv_,
    restore_srv_, reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_, health_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---- configuration ----
  double rate_hz_{50.0};
  std::vector<double> hover_{0.0, 0.0, 3.0};
  double settle_time_{4.0};
  double hover_timeout_{20.0};
  double episode_time_{6.0};
  double service_timeout_{5.0};
  double yaw_T_target_{0.35};
  double yaw_tau_min_{0.15};
  double yaw_tau_max_{1.2};
  std::vector<double> wn_ladder_{1.2, 1.6, 2.0};
  double zeta_target_{0.95};
  double max_change_{1.6};
  double stability_margin_{4.0};
  double consistency_{1.35};
  double min_settle_time_{0.3};
  bool adaptive_episode_{true};
  double min_episode_time_{2.0};
  double episode_settle_periods_{4.0};
  double hover_capture_radius_{0.3};
  double hover_capture_speed_{0.3};
  std::string report_path_{"/tmp/geo_tuner_report.yaml"};
  bool require_enable_{false};
  bool require_offboard_{true};
  std::string offboard_mode_{"OFFBOARD"};

  // ---- state ----
  TuningSchedule sched_;
  SafetyMonitor safety_;
  TunerState state_{TunerState::WAIT_ODOM};
  double state_t0_{0.0};
  std::optional<std::string> px4_mode_;
  std::optional<double> yaw_tau_;   // effective yaw time-constant param
  std::optional<double> attctrl_tau_;  // seeds the in-loop lag prior
  double pre_step_yaw_{0.0};
  double pre_step_pos_{0.0};
  size_t rung_{0};                  // index into wn_ladder_
  double episode_min_time_{0.0};    // earliest adaptive stop [s]
  std::optional<Gains> gains_;
  std::optional<Gains> safe_gains_;
  std::vector<YAML::Node> results_;
  std::string abort_reason_;
  std::string diagnosis_;
  // Acceleration feedforward trim [m/s^2]: cancels steady-state offsets
  // (thrust-map error on z, wind on x/y) the way an integral term would,
  // but transparently and bounded. Sent as the setpoint's acceleration,
  // which the controller uses as feedforward a_ref.
  std::array<double, 3> a_trim_{0.0, 0.0, 0.0};
  int trim_updates_{0};
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
