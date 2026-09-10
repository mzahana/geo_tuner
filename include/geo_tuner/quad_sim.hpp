// Lightweight quadrotor simulator for closed-loop testing of the
// geometric controller + tuning conductor without Gazebo.
//
// Consumes the controller's SE3Command (force [N, inertial], desired
// orientation, desired body rates) and emulates the downstream chain the
// real vehicle has:
//
//   - thrust: the commanded force is projected on the current body z axis
//     (exactly what geometric_mavros_node sends to PX4) and multiplied by
//     a configurable `thrust_scale_error` to emulate a mis-identified
//     thrust map (max_thrust). 1.0 = perfect model.
//   - attitude: body rates track the commanded rates with a first-order
//     lag (`rate_tau`) standing in for the PX4 rate loop.
//   - rigid body: translational dynamics with linear drag; quaternion
//     kinematics.
//   - odometry: published with configurable delay and noise, emulating
//     EKF + mavros transport latency.
//
// This is NOT a replacement for PX4 SITL -- it exists so the conductor's
// logic, safety monitor and identification math run against the *actual
// compiled controller node* in milliseconds-per-test time.
#ifndef GEO_TUNER__QUAD_SIM_HPP_
#define GEO_TUNER__QUAD_SIM_HPP_

#include <Eigen/Dense>
#include <deque>
#include <optional>
#include <random>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <mav_controllers_ros/msg/se3_command.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace geo_tuner
{

class QuadSim : public rclcpp::Node
{
public:
  explicit QuadSim(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SE3Command = mav_controllers_ros::msg::SE3Command;

  double now_s();
  void cmd_cb(const SE3Command::SharedPtr msg);
  void step();
  void queue_odom();
  void release_odom(double now);

  rclcpp::Subscription<SE3Command>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr sim_timer_, odom_timer_;

  double mass_{2.5};
  double rate_tau_{0.06};
  double thrust_scale_{1.0};
  double drag_{0.15};
  Eigen::Vector3d wind_accel_{0.0, 0.0, 0.0};   // steady disturbance [m/s^2]
  double odom_delay_{0.06};
  double pos_noise_{0.003};
  double vel_noise_{0.02};
  double cmd_timeout_{0.5};
  double dt_{1.0 / 500.0};

  Eigen::Vector3d p_{0.0, 0.0, 3.0};
  Eigen::Vector3d v_{Eigen::Vector3d::Zero()};
  Eigen::Vector4d q_{1.0, 0.0, 0.0, 0.0};   // (w, x, y, z)
  Eigen::Vector3d w_{Eigen::Vector3d::Zero()};  // body rates
  std::optional<Eigen::Vector3d> f_cmd_;    // inertial force command [N]
  Eigen::Vector3d w_cmd_{Eigen::Vector3d::Zero()};
  std::optional<double> last_cmd_t_;
  std::mt19937_64 rng_{42};
  std::deque<std::pair<double, nav_msgs::msg::Odometry>> odom_buf_;
};

}  // namespace geo_tuner

#endif  // GEO_TUNER__QUAD_SIM_HPP_
