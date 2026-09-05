"""Fly a synthetic circle so the tracking trails have something to draw:
odometry lags the commanded setpoint, which is exactly the gap the viz
node is meant to show."""
import math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from mav_controllers_ros.msg import TargetCommand

class D(Node):
    def __init__(self):
        super().__init__('moving_smoke')
        self.od = self.create_publisher(Odometry, 'geometric_controller/odom', qos_profile_sensor_data)
        self.sp = self.create_publisher(TargetCommand, 'geometric_controller/setpoint', 10)
        self.mo = self.create_publisher(Bool, 'geometric_controller/enable_motors', 10)
        self.t0 = time.time()
        self.create_timer(0.02, self.tick)
    def tick(self):
        t = time.time() - self.t0
        now = self.get_clock().now().to_msg()
        cx, cy = 3.0*math.cos(0.5*t), 3.0*math.sin(0.5*t)
        # actual lags the command by ~0.25 s -> a visible, growing-with-speed gap
        lag = 0.25
        ax, ay = 3.0*math.cos(0.5*(t-lag)), 3.0*math.sin(0.5*(t-lag))
        o = Odometry(); o.header.stamp = now; o.header.frame_id='map'
        o.pose.pose.position.x = ax; o.pose.pose.position.y = ay
        o.pose.pose.position.z = 3.0; o.pose.pose.orientation.w = 1.0
        self.od.publish(o); self.mo.publish(Bool(data=True))
        c = TargetCommand(); c.header.stamp = now
        c.position.x = cx; c.position.y = cy; c.position.z = 3.0
        self.sp.publish(c)

rclpy.init(); n=D()
t0=time.time()
while time.time()-t0 < 20: rclpy.spin_once(n, timeout_sec=0.02)
rclpy.shutdown()
