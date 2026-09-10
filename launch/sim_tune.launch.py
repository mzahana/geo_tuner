"""Closed-loop tuning test: real geometric controller + lightweight quad
simulator + tuning conductor.

    ros2 launch geo_tuner sim_tune.launch.py
    ros2 launch geo_tuner sim_tune.launch.py thrust_scale_error:=0.75

`thrust_scale_error` emulates a mis-identified thrust map (the plant
produces only e.g. 75% of the force the controller thinks it commands);
the conductor must identify this and converge anyway.

`wind_x/y/z` inject a steady disturbance acceleration [m/s^2] -- the
standing wind + thrust-trim bias the 2026-09-10 field session measured as
[-0.24, -0.22, +0.34]. The T3/T4 acceptance: the SETTLE-phase trim
absorbs it, episodes settle AT the commanded step, and the session
converges to the same gains as a calm run.

`lag_mode:=per_axis` freezes each axis's in-loop lag after its first
clean episode (T2); the acceptance is gain-convergence identical to the
default per_episode run on a calm plant.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    thrust_scale = LaunchConfiguration("thrust_scale_error")
    report_path = LaunchConfiguration("report_path")
    # value_type is required: a bare LaunchConfiguration arrives as a
    # string and the node declares these as doubles.
    step_size = ParameterValue(LaunchConfiguration("step_size"),
                               value_type=float)
    step_size_z = ParameterValue(LaunchConfiguration("step_size_z"),
                                 value_type=float)
    yaw_step = ParameterValue(LaunchConfiguration("yaw_step"),
                              value_type=float)
    wind = {("wind_accel_" + ax): ParameterValue(
        LaunchConfiguration("wind_" + ax), value_type=float)
        for ax in ("x", "y", "z")}

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
            **wind,
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
            # quad_sim has no pilot to hand the vehicle over, so this is
            # the one configuration that uses a fixed hover point -- and
            # even here the setpoint ramps to it rather than stepping.
            "hover_mode": "fixed",
            "hover_position": [0.0, 0.0, 3.0],
            "hover_approach_speed": 0.7,
            # No mavros in this graph: AGL falls back to (odom z - ground_z),
            # and quad_sim's origin IS the ground.
            "agl_topic": "",
            "ground_z": 0.0,
            "min_tuning_altitude": 2.0,
            "step_size": step_size,
            "step_size_z": step_size_z,
            "yaw_step": yaw_step,
            "settle_time": 3.0,
            # episode_time deliberately NOT pinned: the conductor's 8 s
            # default (T3) is part of what this loop exists to exercise.
            "axes": "z,x,y,yaw",
            "wn_ladder": [1.2, 1.6],
            "zeta_target": 0.95,
            "episodes_per_rung": 2,  # median-of-N path, kept short in sim
            "estimate_consistency": 1.35,
            "lag_mode": LaunchConfiguration("lag_mode"),
            "require_offboard": False,  # quad_sim has no mavros/PX4
            "report_path": report_path,
            "safety.min_altitude": 1.0,   # m AGL
            "safety.max_altitude": 20.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("thrust_scale_error", default_value="1.0"),
        DeclareLaunchArgument("report_path",
                              default_value="/tmp/geo_tuner_report.yaml"),
        # Manoeuvre envelope: the vehicle stays within +/- these of the
        # hover point, so they are what a confined space constrains.
        DeclareLaunchArgument("step_size", default_value="0.5"),
        DeclareLaunchArgument("step_size_z", default_value="0.4"),
        DeclareLaunchArgument("yaw_step", default_value="0.5"),
        DeclareLaunchArgument("wind_x", default_value="0.0"),
        DeclareLaunchArgument("wind_y", default_value="0.0"),
        DeclareLaunchArgument("wind_z", default_value="0.0"),
        DeclareLaunchArgument("lag_mode", default_value="per_episode"),
        controller, sim, conductor,
    ])
