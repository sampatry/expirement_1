#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from copy import deepcopy
import time

# Navigation and Message Imports
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient

class PickMissionControl(Node):
    def __init__(self):
        super().__init__('pick_mission_control')
        
        # 1. The Navigator (Handles the TB3 movement)
        self.navigator = BasicNavigator()
        
        # 2. LiDAR Subscriber (Finds the object)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.closest_distance = 99.9  # Initialize with a large number
        
        # 3. Arm Action Client (Controls the OpenManipulator-X)
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def lidar_callback(self, msg):
        """Finds the closest object in a 180-degree arc in front of the robot."""
        # Indices 0-90 and 270-359 are roughly the front 180 degrees
        front_indices = list(range(0, 90)) + list(range(270, 359))
        available_distances = [msg.ranges[i] for i in front_indices if msg.ranges[i] > 0.1]
        
        if available_distances:
            self.closest_distance = min(available_distances)

    def move_arm_to_pick(self):
        """Moves the TurtleBot3 arm to a 'reach and grab' pose."""
        self.get_logger().info('Sending Pick command to arm...')
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        point = JointTrajectoryPoint()
        # Radian values for a 'reach forward' pose (example values)
        point.positions = [0.0, 0.4, 0.2, -0.1] 
        point.time_from_start.sec = 3
        
        goal_msg.trajectory.points = [point]
        self.arm_client.wait_for_server()
        return self.arm_client.send_goal_async(goal_msg)

    def run(self):
        # --- A. INITIAL POSE ---
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.50
        initial_pose.pose.position.y = 0.90
        initial_pose.pose.orientation.w = 1.0
        
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

        # --- B. DEFINE & NAVIGATE ROUTE ---
        inspection_route = [[0.5, 0.3], [2.0, 1.0], [3.0, 0.3]]
        inspection_points = []
        
        tmp_pose = PoseStamped()
        tmp_pose.header.frame_id = 'map'
        for pt in inspection_route:
            tmp_pose.pose.position.x = pt[0]
            tmp_pose.pose.position.y = pt[1]
            inspection_points.append(deepcopy(tmp_pose))

        print("Starting inspection route...")
        self.navigator.followWaypoints(inspection_points)

        # While moving, we must 'spin' so the lidar_callback works
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        # --- C. THE PICK (At the final waypoint) ---
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            print(f"Arrived! Object detected at {self.closest_distance:.2f}m. Picking up...")
            self.move_arm_to_pick()
            time.sleep(5.0) # Wait for arm to finish moving

        # --- D. RETURN HOME ---
        print("Returning to start...")
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.goToPose(initial_pose)
        
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        print("Mission Complete.")

def main():
    rclpy.init()
    mission = PickMissionControl()
    mission.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()