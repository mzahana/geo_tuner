#!/usr/bin/env python3
# Copyright (c) 2026 Mohamed Abdelkader
# SPDX-License-Identifier: MIT
"""Draw commanded vs actual tracking in RViz.

The geometric controller publishes its commanded pose as a PoseStamped
(geometric_controller/cmd_pose) and RViz has no trail display for one -- you
get a dot and no history. This node keeps a rolling trail of the command and
of the measured odometry as nav_msgs/Path, plus a line joining the two so the
tracking error is visible as a gap rather than inferred from numbers.

Pure consumer: it subscribes and republishes for display. It never publishes
a setpoint, calls a service, or writes a parameter, so nothing it does can
reach the flight controller.

    ros2 run geo_tuner tracking_viz.py --ros-args -r __ns:=/interceptor
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


class TrackingViz(Node):
    def __init__(self):
        super().__init__('tracking_viz')

        self.declare_parameter('max_points', 3000)
        self.declare_parameter('min_point_dist', 0.05)   # [m] trail decimation
        self.declare_parameter('publish_rate', 10.0)     # [Hz]
        self.declare_parameter('error_scale', 0.03)      # [m] error line width
        # The controller stamps its poses with the odometry frame; this only
        # overrides that when a message arrives without a frame_id.
        self.declare_parameter('fallback_frame', 'map')

        self.max_points = int(self.get_parameter('max_points').value)
        self.min_dist = float(self.get_parameter('min_point_dist').value)
        self.error_scale = float(self.get_parameter('error_scale').value)
        self.fallback_frame = str(self.get_parameter('fallback_frame').value)

        self.cmd_path = Path()
        self.actual_path = Path()
        self.cmd_pose = None
        self.actual_pose = None

        self.create_subscription(PoseStamped, 'geometric_controller/cmd_pose',
                                 self.cmd_cb, 10)
        # The actual pose comes from the odometry the controller is closing
        # the loop on -- geometric_controller/odom_pose exists as a publisher
        # in the controller but is never published, so subscribing to it
        # would leave the actual trail permanently empty.
        self.create_subscription(Odometry, 'geometric_controller/odom',
                                 self.actual_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Path, 'viz/command_path', 10)
        self.actual_pub = self.create_publisher(Path, 'viz/actual_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'viz/tracking', 10)

        # Trails accumulate across a whole flight; clearing them between runs
        # keeps a comparison readable.
        self.create_service(Empty, '~/reset', self.reset_cb)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 0.1), self.publish)

        self.get_logger().info(
            'tracking_viz up: cmd_pose/odom_pose -> viz/command_path, '
            'viz/actual_path, viz/tracking (call ~/reset to clear trails)')

    # ------------------------------------------------------------------
    def _frame(self, msg):
        return msg.header.frame_id if msg.header.frame_id else self.fallback_frame

    def _append(self, path, msg):
        # Decimate by distance: at 50 Hz an undecimated trail is tens of
        # thousands of points within a minute, which stalls RViz.
        if path.poses:
            last = path.poses[-1].pose.position
            p = msg.pose.position
            if math.dist((last.x, last.y, last.z), (p.x, p.y, p.z)) < self.min_dist:
                return
        path.header.frame_id = self._frame(msg)
        path.poses.append(msg)
        if len(path.poses) > self.max_points:
            del path.poses[0:len(path.poses) - self.max_points]

    def cmd_cb(self, msg):
        self.cmd_pose = msg
        self._append(self.cmd_path, msg)

    def actual_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.actual_pose = pose
        self._append(self.actual_path, pose)

    def reset_cb(self, _request, response):
        self.cmd_path.poses.clear()
        self.actual_path.poses.clear()
        self.get_logger().info('trails cleared')
        return response

    # ------------------------------------------------------------------
    def publish(self):
        now = self.get_clock().now().to_msg()

        for path, pub in ((self.cmd_path, self.cmd_pub),
                          (self.actual_path, self.actual_pub)):
            if path.poses:
                path.header.stamp = now
                pub.publish(path)

        if self.cmd_pose is None or self.actual_pose is None:
            return

        frame = self._frame(self.actual_pose)
        c = self.cmd_pose.pose.position
        a = self.actual_pose.pose.position
        err = math.dist((c.x, c.y, c.z), (a.x, a.y, a.z))

        markers = MarkerArray()

        # The error itself, drawn as the gap between where it is and where
        # it was told to be.
        line = Marker()
        line.header.frame_id = frame
        line.header.stamp = now
        line.ns = 'tracking_error'
        line.id = 0
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD
        line.scale.x = self.error_scale
        line.pose.orientation.w = 1.0
        line.points = [Point(x=a.x, y=a.y, z=a.z), Point(x=c.x, y=c.y, z=c.z)]
        # Green under 10 cm, amber to 50 cm, red beyond: the same bands the
        # health panel uses to judge a gain change.
        if err < 0.1:
            line.color.r, line.color.g, line.color.b = 0.25, 0.8, 0.35
        elif err < 0.5:
            line.color.r, line.color.g, line.color.b = 0.9, 0.65, 0.2
        else:
            line.color.r, line.color.g, line.color.b = 0.9, 0.25, 0.25
        line.color.a = 0.9
        markers.markers.append(line)

        text = Marker()
        text.header.frame_id = frame
        text.header.stamp = now
        text.ns = 'tracking_error'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = 0.5 * (a.x + c.x)
        text.pose.position.y = 0.5 * (a.y + c.y)
        text.pose.position.z = 0.5 * (a.z + c.z) + 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = 0.35
        text.color = line.color
        text.text = f'{err:.2f} m'
        markers.markers.append(text)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TrackingViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
