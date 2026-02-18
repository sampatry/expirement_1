import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('navigation_pkg'), 'config')
    map_file = os.path.join(config_dir, 'my_map.yaml')

    #runs this command: ros2 launch turtlebot3_gazebo turtlebot3_epirement1.launch.py
    simulation_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_epirement1.launch.py'
    )

    #launches nav2
    nav2_launch = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
            }.items()
        ),
    ])