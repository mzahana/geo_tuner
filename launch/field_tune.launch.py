"""Field tuning session: launches ONLY the tuning conductor.

The geometric controller + mavros must already be running (your normal
bringup), the vehicle hovering in OFFBOARD near the configured hover
position, with the pilot ready on the RC mode switch.

    ros2 launch geo_tuner field_tune.launch.py
    ros2 launch geo_tuner field_tune.launch.py params:=/path/to/tuner_field.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("geo_tuner"), "config", "tuner_field.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        Node(
            package="geo_tuner",
            executable="tuning_conductor",
            name="tuning_conductor",
            output="screen",
            parameters=[LaunchConfiguration("params")],
        ),
    ])
