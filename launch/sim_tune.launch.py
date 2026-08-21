"""Closed-loop tuning test: real geometric controller + lightweight quad
simulator + tuning conductor.

    ros2 launch geo_tuner sim_tune.launch.py
    ros2 launch geo_tuner sim_tune.launch.py thrust_scale_error:=0.75

`thrust_scale_error` emulates a mis-identified thrust map (the plant
produces only e.g. 75% of the force the controller thinks it commands);
the conductor must identify this and converge anyway.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    thrust_scale = LaunchConfiguration("thrust_scale_error")
    report_path = LaunchConfiguration("report_path")

    controller = Node(
        package="mav_controllers_ros",
        executable="geometric_controller_node",
        name="geometric_controller_node",
        output="screen",
        parameters=[{
            "mass": 2.5,
            "use_external_yaw": True,
            # deliberately conservative / slightly wrong starting gains:
            # the conductor has to identify and fix them
            "gains.pos.x": 2.0, "gains.pos.y": 2.0, "gains.pos.z": 3.0,
            "gains.vel.x": 2.7, "gains.vel.y": 2.7, "gains.vel.z": 3.3,
            "gains.ki.x": 0.0, "gains.ki.y": 0.0, "gains.ki.z": 0.0,
            "attctrl_tau": 0.3,
            "max_tilt_angle": 0.52,
            "max_accel": 5.0,
        }],
    )

    sim = Node(
        package="geo_tuner",
        executable="quad_sim",
        name="quad_sim",
        output="screen",
        parameters=[{
            "mass": 2.5,
            "thrust_scale_error": thrust_scale,
            "rate_tau": 0.06,
            "odom_delay": 0.06,
            "start_position": [0.0, 0.0, 3.0],
        }],
    )

    conductor = Node(
        package="geo_tuner",
        executable="tuning_conductor",
        name="tuning_conductor",
        output="screen",
        parameters=[{
            "controller_node": "geometric_controller_node",
            "hover_position": [0.0, 0.0, 3.0],
            "step_size": 0.5,
            "step_size_z": 0.4,
            "settle_time": 3.0,
            "episode_time": 6.0,
            "axes": "z,x,y,yaw",
            "wn_ladder": [1.2, 1.6],
            "zeta_target": 0.95,
            "episodes_per_rung": 2,  # median-of-N path, kept short in sim
            "estimate_consistency": 1.35,
            "require_offboard": False,  # quad_sim has no mavros/PX4
            "report_path": report_path,
            "safety.min_altitude": 1.0,
            "safety.max_altitude": 20.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("thrust_scale_error", default_value="1.0"),
        DeclareLaunchArgument("report_path",
                              default_value="/tmp/geo_tuner_report.yaml"),
        controller, sim, conductor,
    ])
