#!/usr/bin/env python3

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from launch.actions import TimerAction


def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    inspection_route = [
        # [0.5, 0.3],
        # [2.0, 1.0],
        [3.0, 0.3],
    ]

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.50
    initial_pose.pose.position.y = 0.90
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Send our route
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    for pt in inspection_route:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        inspection_points.append(deepcopy(inspection_pose))

    wpf_task = navigator.followWaypoints(inspection_points)

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstration
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(inspection_points))
            )

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print(f'Inspection of shelving failed!:{error_code}:{error_msg}')
        print('Returning to start...')

    # go back to start
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # go_to_pose_task = navigator.goToPose(initial_pose)
    # while not navigator.isTaskComplete():
    #     pass

    exit(0)


if __name__ == '__main__':
    main()