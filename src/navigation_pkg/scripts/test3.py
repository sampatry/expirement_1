#!/usr/bin/env python3

from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    # Define inspection route as [x, y, yaw_degrees]
    inspection_route = [
        [0.5, 0.3, 0],
        [2.0, 1.0, 0],
        [3.0, 0.7, 0],
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

    # Wait for Nav2 to activate
    navigator.waitUntilNav2Active()

    # Build continuous path
    path = []
    for pt in inspection_route:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]

        # Convert yaw to quaternion
        yaw_rad = math.radians(pt[2])
        q = quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        path.append(deepcopy(pose))

    # Send continuous path to follow
    path_task = navigator.followPath(path)

    # Monitor execution
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                f"Following path: waypoint {feedback.current_waypoint + 1}/{len(path)}"
            )

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Inspection complete!")
    elif result == TaskResult.CANCELED:
        print("Inspection canceled.")
    elif result == TaskResult.FAILED:
        error_code, error_msg = navigator.getTaskError()
        print(f"Inspection failed: {error_code}:{error_msg}")

    # Exit ROS
    exit(0)


if __name__ == "__main__":
    main()