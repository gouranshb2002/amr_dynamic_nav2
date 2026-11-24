import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = get_package_share_directory('amr_simulation')
    pkg_up = get_package_share_directory('amr_bringup')
    world = os.path.join(pkg_sim, 'worlds', 'dynamic_warehouse.world')
    gui = LaunchConfiguration('gui', default='false')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_up, 'launch', 'phase2_nav2.launch.py')),
            launch_arguments={'world': world, 'gui': gui}.items(),
        ),
        Node(package='amr_simulation', executable='dynamic_obstacle_manager.py', name='dynamic_obstacle_manager'),
    ])
