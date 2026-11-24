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
    world = LaunchConfiguration('world', default=os.path.join(pkg_sim, 'worlds', 'dev_warehouse.world'))
    map_yaml = LaunchConfiguration('map', default=os.path.join(pkg_sim, 'maps', 'dev_warehouse.yaml'))
    gui = LaunchConfiguration('gui', default='false')
    nav2_cfg = os.path.join(pkg_up, 'config', 'nav2_params.yaml')
    wd_cfg = os.path.join(pkg_up, 'config', 'watchdog.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world),
        DeclareLaunchArgument('map', default_value=map_yaml),
        DeclareLaunchArgument('gui', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_sim, 'launch', 'sim_robot.launch.py')),
            launch_arguments={'world': world, 'gui': gui}.items(),
        ),
        Node(package='amr_bringup', executable='cmd_vel_watchdog', name='cmd_vel_watchdog', parameters=[wd_cfg]),
        Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[nav2_cfg, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', parameters=[nav2_cfg]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', parameters=[nav2_cfg], remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', parameters=[nav2_cfg]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', parameters=[nav2_cfg]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', parameters=[nav2_cfg]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', parameters=[nav2_cfg], remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', parameters=[nav2_cfg]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager', parameters=[nav2_cfg]),
    ])
