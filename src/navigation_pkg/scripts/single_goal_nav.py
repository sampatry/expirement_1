#!/usr/bin/env python3

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler

def compute_heading(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

def create_pose(x, y, theta=0.0, frame='map', navigator=None):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    q = quaternion_from_euler(0, 0, theta)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()
    nav_start = navigator.get_clock().now()

    # Set initial pose
    initial_pose = create_pose(0.50, 0.90, 0.0, navigator=navigator)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # Define raw waypoints (x, y)
    raw_points = [
        (1.9, 3.7),
        (1.27, 3.2),
    ]

    # Generate poses with heading
    goal_poses = []
    prev_x, prev_y = initial_pose.pose.position.x, initial_pose.pose.position.y
    for x, y in raw_points:
        heading = compute_heading(prev_x, prev_y, x, y)
        goal_poses.append(create_pose(x, y, heading, navigator=navigator))
        prev_x, prev_y = x, y

    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(goal_poses)}')
            now = navigator.get_clock().now()
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()
