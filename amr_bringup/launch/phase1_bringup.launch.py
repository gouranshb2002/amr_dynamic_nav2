import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_amr_sim = get_package_share_directory('amr_simulation')
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    watchdog_cfg = os.path.join(pkg_amr_bringup, 'config', 'watchdog.yaml')
    gui = LaunchConfiguration('gui', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_amr_sim, 'launch', 'sim_robot.launch.py')),
            launch_arguments={'gui': gui}.items(),
        ),
        Node(
            package='amr_bringup',
            executable='cmd_vel_watchdog',
            name='cmd_vel_watchdog',
            output='screen',
            parameters=[watchdog_cfg],
        ),
    ])
