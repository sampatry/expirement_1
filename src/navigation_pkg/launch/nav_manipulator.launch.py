import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_file = os.path.join(
        get_package_share_directory('navigation_pkg'), 'config', 'my_map.yaml'
    )

    param_file = os.path.join(
        get_package_share_directory('navigation_pkg'), 'config', 'turtlebot3.yaml'
    )

    nav2_launch = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_navigation2'),
        'launch', 'navigation2.launch.py'
    )

    moveit_launch_path = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_moveit_config'),
        'launch', 'move_group.launch.py'
    )

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(sim_launch)
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map_yaml_file': map_file,
                'params_file': param_file,
                'use_sim': 'true',
                'start_rviz': 'true',
            }.items()
        ),
        # This starts the /move_action server your Python script is looking for
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_path),
            launch_arguments={
                'use_sim_time': 'true',
            }.items()
        ),
    ])