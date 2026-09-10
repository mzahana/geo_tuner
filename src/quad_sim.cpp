#include "geo_tuner/quad_sim.hpp"

#include <algorithm>
#include <cmath>

#include "geo_tuner/core/gain_design.hpp"   // kGravity

namespace geo_tuner
{
namespace
{

Eigen::Vector4d quat_mult(const Eigen::Vector4d & q1, const Eigen::Vector4d & q2)
{
  const double w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
  const double w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];
  return {
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2};
}

Eigen::Matrix3d quat_to_R(const Eigen::Vector4d & q)
{
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  Eigen::Matrix3d R;
  R << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
  return R;
}

}  // namespace

QuadSim::QuadSim(const rclcpp::NodeOptions & options)
: rclcpp::Node("quad_sim", options)
{
  mass_ = declare_parameter<double>("mass", 2.5);
  const double sim_rate = declare_parameter<double>("sim_rate", 500.0);
  const double odom_rate = declare_parameter<double>("odom_rate", 100.0);
  odom_delay_ = declare_parameter<double>("odom_delay", 0.06);        // s
  pos_noise_ = declare_parameter<double>("odom_pos_noise", 0.003);    // m std
  vel_noise_ = declare_parameter<double>("odom_vel_noise", 0.02);     // m/s std
  rate_tau_ = declare_parameter<double>("rate_tau", 0.06);            // s, PX4 rate-loop lag
  thrust_scale_ = declare_parameter<double>("thrust_scale_error", 1.0);
  drag_ = declare_parameter<double>("drag_coeff", 0.15);              // N per m/s
  // Constant disturbance acceleration [m/s^2], inertial frame: steady
  // wind push + thrust-map bias, the combination the 2026-09-10 field
  // session measured as [-0.24, -0.22, +0.34]. Exists so the T3/T4
  // acceptance (bias absorbed by the trim, episodes settle AT the target)
  // can be flown in sim.
  wind_accel_ = Eigen::Vector3d(
    declare_parameter<double>("wind_accel_x", 0.0),
    declare_parameter<double>("wind_accel_y", 0.0),
    declare_parameter<double>("wind_accel_z", 0.0));
  const auto start =
    declare_parameter<std::vector<double>>("start_position", {0.0, 0.0, 3.0});
  cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
  dt_ = 1.0 / sim_rate;

  p_ = Eigen::Vector3d(start[0], start[1], start[2]);

  cmd_sub_ = create_subscription<SE3Command>(
    "geometric_controller/cmd", 10,
    std::bind(&QuadSim::cmd_cb, this, std::placeholders::_1));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "geometric_controller/odom", rclcpp::SensorDataQoS());

  sim_timer_ = create_wall_timer(
    std::chrono::duration<double>(dt_), std::bind(&QuadSim::step, this));
  odom_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / odom_rate),
    std::bind(&QuadSim::queue_odom, this));
  RCLCPP_INFO(
    get_logger(),
    "quad_sim: mass=%g thrust_scale_error=%g rate_tau=%g odom_delay=%g",
    mass_, thrust_scale_, rate_tau_, odom_delay_);
}

double QuadSim::now_s()
{
  return static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
}

void QuadSim::cmd_cb(const SE3Command::SharedPtr msg)
{
  f_cmd_ = Eigen::Vector3d(msg->force.x, msg->force.y, msg->force.z);
  w_cmd_ = Eigen::Vector3d(
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  last_cmd_t_ = now_s();
}

void QuadSim::step()
{
  const double now = now_s();
  const double dt = dt_;

  const bool have_cmd = f_cmd_ && last_cmd_t_ && (now - *last_cmd_t_ < cmd_timeout_);
  if (!have_cmd) {
    // On the "ground"/pre-offboard: hold perfectly still (vehicle is
    // assumed hovering under PX4 position mode before handover)
    v_.setZero();
    w_.setZero();
    release_odom(now);
    return;
  }

  // Rate loop: first-order tracking of commanded body rates
  w_ += (w_cmd_ - w_) * (dt / std::max(rate_tau_, dt));

  // Attitude kinematics
  const Eigen::Vector4d omega(0.0, w_[0], w_[1], w_[2]);
  const Eigen::Vector4d dq = 0.5 * quat_mult(q_, omega);
  q_ += dq * dt;
  q_.normalize();
  const Eigen::Matrix3d R = quat_to_R(q_);

  // Thrust: project commanded inertial force on current body z
  double thrust = thrust_scale_ * f_cmd_->dot(R.col(2));
  thrust = std::max(0.0, thrust);
  const Eigen::Vector3d acc = (thrust / mass_) * R.col(2) -
    Eigen::Vector3d(0.0, 0.0, kGravity) - (drag_ / mass_) * v_ + wind_accel_;

  v_ += acc * dt;
  p_ += v_ * dt;
  release_odom(now);
}

void QuadSim::queue_odom()
{
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  std::normal_distribution<double> pn(0.0, pos_noise_), vn(0.0, vel_noise_);
  msg.pose.pose.position.x = p_[0] + pn(rng_);
  msg.pose.pose.position.y = p_[1] + pn(rng_);
  msg.pose.pose.position.z = p_[2] + pn(rng_);
  msg.pose.pose.orientation.w = q_[0];
  msg.pose.pose.orientation.x = q_[1];
  msg.pose.pose.orientation.y = q_[2];
  msg.pose.pose.orientation.z = q_[3];
  msg.twist.twist.linear.x = v_[0] + vn(rng_);
  msg.twist.twist.linear.y = v_[1] + vn(rng_);
  msg.twist.twist.linear.z = v_[2] + vn(rng_);
  msg.twist.twist.angular.x = w_[0];
  msg.twist.twist.angular.y = w_[1];
  msg.twist.twist.angular.z = w_[2];
  odom_buf_.emplace_back(now_s() + odom_delay_, msg);
}

void QuadSim::release_odom(double now)
{
  while (!odom_buf_.empty() && odom_buf_.front().first <= now) {
    odom_pub_->publish(odom_buf_.front().second);
    odom_buf_.pop_front();
  }
}

}  // namespace geo_tuner
