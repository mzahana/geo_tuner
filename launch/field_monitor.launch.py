"""RViz with the geometric-controller field panels.

    ros2 launch geo_tuner field_monitor.launch.py
    ros2 launch geo_tuner field_monitor.launch.py ns:=interceptor

`ns` only pre-fills the panel's namespace box; it can also be typed into the
panel at runtime and is stored in the saved RViz config. The panels are pure
consumers -- launching this cannot affect the vehicle.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz = os.path.join(
        get_package_share_directory("geo_tuner"), "rviz", "geo_field.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument(
            "tracking_viz", default_value="true",
            description="Also run tracking_viz, which turns the controller's "
                        "commanded and measured poses into Paths plus an error "
                        "line for the 3D view. Pure consumer."),
        DeclareLaunchArgument(
            "ns", default_value="",
            description="Namespace of the controller stack, e.g. 'interceptor' "
                        "in the d2dtracker SITL. Empty for a bare field stack."),
        Node(
            package="rviz2",
            executable="rviz2",
            name="geo_field_rviz",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            # The panels read this namespace when their own box is empty,
            # so `ns:=interceptor` needs no typing at the panel.
            namespace=LaunchConfiguration("ns"),
        ),
        Node(
            package="geo_tuner",
            executable="tracking_viz.py",
            name="tracking_viz",
            output="screen",
            namespace=LaunchConfiguration("ns"),
            condition=IfCondition(LaunchConfiguration("tracking_viz")),
        ),
    ])
