"""Lightweight quadrotor simulator for closed-loop testing of the
geometric controller + tuning conductor without Gazebo.

Consumes the controller's SE3Command (force [N, inertial], desired
orientation, desired body rates) and emulates the downstream chain the
real vehicle has:

  - thrust: the commanded force is projected on the current body z axis
    (exactly what geometric_mavros_node sends to PX4) and multiplied by a
    configurable `thrust_scale_error` to emulate a mis-identified thrust
    map (max_thrust). 1.0 = perfect model.
  - attitude: body rates track the commanded rates with a first-order lag
    (`rate_tau`) standing in for the PX4 rate loop.
  - rigid body: translational dynamics with linear drag; quaternion
    kinematics.
  - odometry: published with configurable delay and noise, emulating
    EKF + mavros transport latency.

This is NOT a replacement for PX4 SITL — it exists so the conductor's
logic, safety monitor and identification math run against the *actual
compiled controller node* in milliseconds-per-test time.
"""

from __future__ import annotations

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from mav_controllers_ros.msg import SE3Command

G = 9.81


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class QuadSim(Node):

    def __init__(self):
        super().__init__("quad_sim")
        self.declare_parameter("mass", 2.5)
        self.declare_parameter("sim_rate", 500.0)
        self.declare_parameter("odom_rate", 100.0)
        self.declare_parameter("odom_delay", 0.06)       # s
        self.declare_parameter("odom_pos_noise", 0.003)  # m std
        self.declare_parameter("odom_vel_noise", 0.02)   # m/s std
        self.declare_parameter("rate_tau", 0.06)         # s, PX4 rate-loop lag
        self.declare_parameter("thrust_scale_error", 1.0)
        self.declare_parameter("drag_coeff", 0.15)       # N per m/s
        self.declare_parameter("start_position", [0.0, 0.0, 3.0])
        self.declare_parameter("cmd_timeout", 0.5)

        gp = lambda n: self.get_parameter(n).value
        self.mass = float(gp("mass"))
        self.rate_tau = float(gp("rate_tau"))
        self.thrust_scale = float(gp("thrust_scale_error"))
        self.drag = float(gp("drag_coeff"))
        self.odom_delay = float(gp("odom_delay"))
        self.pos_noise = float(gp("odom_pos_noise"))
        self.vel_noise = float(gp("odom_vel_noise"))
        self.cmd_timeout = float(gp("cmd_timeout"))
        self.dt = 1.0 / float(gp("sim_rate"))

        self.p = np.array(gp("start_position"), dtype=float)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)          # body rates
        self.f_cmd = None             # inertial force command [N]
        self.w_cmd = np.zeros(3)
        self.last_cmd_t = None
        self.rng = np.random.default_rng(42)
        self._odom_buf: deque = deque()  # (t_release, Odometry)

        self.cmd_sub = self.create_subscription(
            SE3Command, "geometric_controller/cmd", self._cmd_cb, 10)
        self.odom_pub = self.create_publisher(
            Odometry, "geometric_controller/odom", qos_profile_sensor_data)

        self.sim_timer = self.create_timer(self.dt, self._step)
        self.odom_timer = self.create_timer(1.0 / float(gp("odom_rate")),
                                            self._queue_odom)
        self.get_logger().info(
            f"quad_sim: mass={self.mass} thrust_scale_error={self.thrust_scale} "
            f"rate_tau={self.rate_tau} odom_delay={self.odom_delay}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_cb(self, msg: SE3Command):
        self.f_cmd = np.array([msg.force.x, msg.force.y, msg.force.z])
        self.w_cmd = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                               msg.angular_velocity.z])
        self.last_cmd_t = self._now()

    def _step(self):
        now = self._now()
        dt = self.dt

        have_cmd = (self.f_cmd is not None and self.last_cmd_t is not None
                    and now - self.last_cmd_t < self.cmd_timeout)
        if not have_cmd:
            # On the "ground"/pre-offboard: hold perfectly still (vehicle
            # is assumed hovering under PX4 position mode before handover)
            self.v[:] = 0.0
            self.w[:] = 0.0
            self._release_odom(now)
            return

        # Rate loop: first-order tracking of commanded body rates
        self.w += (self.w_cmd - self.w) * (dt / max(self.rate_tau, dt))

        # Attitude kinematics
        dq = 0.5 * quat_mult(self.q, np.array([0.0, *self.w]))
        self.q = self.q + dq * dt
        self.q /= np.linalg.norm(self.q)
        R = quat_to_R(self.q)

        # Thrust: project commanded inertial force on current body z
        thrust = self.thrust_scale * float(self.f_cmd @ R[:, 2])
        thrust = max(0.0, thrust)
        acc = (thrust / self.mass) * R[:, 2] - np.array([0.0, 0.0, G]) \
            - (self.drag / self.mass) * self.v

        self.v += acc * dt
        self.p += self.v * dt
        self._release_odom(now)

    def _queue_odom(self):
        now = self._now()
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        pn = self.rng.normal(0.0, self.pos_noise, 3)
        vn = self.rng.normal(0.0, self.vel_noise, 3)
        msg.pose.pose.position.x = self.p[0] + pn[0]
        msg.pose.pose.position.y = self.p[1] + pn[1]
        msg.pose.pose.position.z = self.p[2] + pn[2]
        msg.pose.pose.orientation.w = self.q[0]
        msg.pose.pose.orientation.x = self.q[1]
        msg.pose.pose.orientation.y = self.q[2]
        msg.pose.pose.orientation.z = self.q[3]
        msg.twist.twist.linear.x = self.v[0] + vn[0]
        msg.twist.twist.linear.y = self.v[1] + vn[1]
        msg.twist.twist.linear.z = self.v[2] + vn[2]
        msg.twist.twist.angular.x = self.w[0]
        msg.twist.twist.angular.y = self.w[1]
        msg.twist.twist.angular.z = self.w[2]
        self._odom_buf.append((self._now() + self.odom_delay, msg))

    def _release_odom(self, now):
        while self._odom_buf and self._odom_buf[0][0] <= now:
            _, msg = self._odom_buf.popleft()
            self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QuadSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
