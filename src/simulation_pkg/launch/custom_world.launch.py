#!/usr/bin/env python3

#launches turtlebot with open manipulator

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim = get_package_share_directory('simulation_pkg')

    world_path = os.path.join(pkg_sim, 'worlds', 'turtlebot3_expirement1.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.3')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    roll  = LaunchConfiguration('roll',  default='0.00')
    pitch = LaunchConfiguration('pitch', default='0.00')
    yaw = LaunchConfiguration('yaw', default='0.0')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Launches the manipulator base (joint state publisher, controllers, etc.)
    manipulator_base_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_manipulation_gazebo'),
                'launch',
                'base.launch.py'
            ])
        ),
        launch_arguments={
            'start_rviz': 'false',
            'prefix': '""',
            'use_sim': 'true',
        }.items()
    )

    # Spawns the combined turtlebot3 + manipulator entity
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'turtlebot3_manipulation_system',
            '-x', x_pose, '-y', y_pose, '-z', z_pose,
            '-R', roll, '-P', pitch, '-Y', yaw,
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(pkg_sim, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    ))
    
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(manipulator_base_cmd)
    ld.add_action(spawn_entity_cmd)
    return ld