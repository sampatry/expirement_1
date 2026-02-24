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

    # sim_launch = os.path.join(
    #     get_package_share_directory('simulation_pkg'), 'launch', 'custom_world.launch.py'
    # )

    nav2_launch = os.path.join(
        get_package_share_directory('turtlebot3_manipulation_navigation2'),
        'launch', 'navigation2.launch.py'
    )

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(sim_launch)
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map_yaml_file': map_file,
                'use_sim': 'true',
                'start_rviz': 'true',
            }.items()
        ),
    ])