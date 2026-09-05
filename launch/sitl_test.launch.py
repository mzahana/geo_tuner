"""Everything except the simulator, on one machine, for testing in SITL.

In the field the vehicle side and the ground station are separate machines and
you use the two launches separately:

    vehicle : ros2 launch mav_controllers_ros panel_support.launch.py controller_ns:=interceptor
    laptop  : ros2 launch geo_tuner field_monitor.launch.py ns:=interceptor

In SITL both live on this machine, so this starts both at once. It does NOT
start the simulator or the controller -- bring those up first, the way you
normally do:

    ros2 launch d2dtracker_sim sitl_bringup.launch.py with_controller:=true
    ros2 launch geo_tuner sitl_test.launch.py ns:=interceptor

Then take off and switch to OFFBOARD, and drive everything from the panels.
Nothing here commands the vehicle on its own: trajectory_test_node comes up
holding, with auto_start off.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    panels = get_package_share_directory('geo_tuner')
    controllers = get_package_share_directory('mav_controllers_ros')
    tuner = get_package_share_directory('geo_tuner')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ns', default_value='interceptor',
            description="Namespace of the controller stack. 'interceptor' is the "
                        "d2dtracker SITL default; pass an empty string for a bare "
                        "stack at the root."),
        # Deliberately NOT called 'rviz'. IncludeLaunchDescription does not
        # scope launch configurations, so a name shared with an included file
        # is set in both directions: 'rviz' here would have opened
        # trajectory_test's own RViz as well, and the 'rviz:=false' that file
        # is given would have come back and closed this one.
        DeclareLaunchArgument(
            'open_rviz', default_value='true',
            description='Open RViz with the field panels.'),
        DeclareLaunchArgument(
            'tuner', default_value='true',
            description='Run the geo_tuner conductor that the Tuner tab drives. '
                        'It flies nothing until START is pressed.'),
        DeclareLaunchArgument(
            'trajectory_type', default_value='',
            description='Initial shape for trajectory_test_node (empty keeps the '
                        'YAML value); the panel can change it at any time.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(controllers, 'launch', 'panel_support.launch.py')),
            launch_arguments={
                'controller_ns': LaunchConfiguration('ns'),
                'trajectory_type': LaunchConfiguration('trajectory_type'),
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tuner, 'launch', 'field_tune.launch.py')),
            launch_arguments={
                'ns': LaunchConfiguration('ns'),
                'require_enable': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('tuner')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(panels, 'launch', 'field_monitor.launch.py')),
            launch_arguments={'ns': LaunchConfiguration('ns')}.items(),
            condition=IfCondition(LaunchConfiguration('open_rviz')),
        ),
    ])
