"""Field tuning session: launches ONLY the tuning conductor.

The geometric controller + mavros must already be running (your normal
bringup), the vehicle hovering in OFFBOARD near the configured hover
position, with the pilot ready on the RC mode switch.

    ros2 launch geo_tuner field_tune.launch.py
    ros2 launch geo_tuner field_tune.launch.py params:=/path/to/tuner_field.yaml
    ros2 launch geo_tuner field_tune.launch.py ns:=interceptor   # d2dtracker SITL
    ros2 launch geo_tuner field_tune.launch.py require_enable:=true  # panel START
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("geo_tuner"), "config", "tuner_field.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument(
            "require_enable", default_value="false",
            description="Hold at WAIT_ENABLE until the ~/start service is "
                        "called (the RViz Tuner panel's START button), instead "
                        "of beginning the moment PX4 enters OFFBOARD. Pass "
                        "true when driving the session from the panel: the "
                        "session then cannot begin from a mode switch alone."),
        DeclareLaunchArgument(
            "ns", default_value="",
            description="Namespace of the controller stack (e.g. 'interceptor' "
                        "in the d2dtracker sim). Topics and the parameter "
                        "client follow it."),
        DeclareLaunchArgument(
            "output_dir", default_value="",
            description="Directory for session data. Every session writes a "
                        "timestamped report YAML plus a folder of raw "
                        "per-episode CSVs here, so repeated sessions never "
                        "overwrite each other. Empty keeps the YAML's "
                        "report_path / episode_dump_dir behaviour."),
        Node(
            package="geo_tuner",
            executable="tuning_conductor",
            name="tuning_conductor",
            namespace=LaunchConfiguration("ns"),
            output="screen",
            parameters=[
                LaunchConfiguration("params"),
                # With a namespace, the controller node lives under it too;
                # relative topics already follow the namespace.
                {"controller_node": [LaunchConfiguration("ns"),
                                     "/geometric_controller_node"],
                 "require_enable": ParameterValue(
                     LaunchConfiguration("require_enable"), value_type=bool),
                 "output_dir": ParameterValue(
                     LaunchConfiguration("output_dir"), value_type=str)},
            ],
        ),
    ])
