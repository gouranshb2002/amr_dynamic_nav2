import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_amr_sim = get_package_share_directory('amr_simulation')
    pkg_amr_desc = get_package_share_directory('amr_description')
    default_world = os.path.join(pkg_amr_sim, 'worlds', 'phase1_calibration.world')
    world = LaunchConfiguration('world', default=default_world)
    gui = LaunchConfiguration('gui', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_amr_desc, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'amr_bot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.01', '-Y', '0.0'],
            output='screen',
        ),
    ])
