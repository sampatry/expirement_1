#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from copy import deepcopy
import math
import time

# Navigation and Message Imports
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# MoveIt2 Action Imports
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive

class TB3FinalMission(Node):
    def __init__(self):
        # Initialize as a ROS 2 Node so we can use Subscribers/ActionClients
        super().__init__('tb3_final_mission')

        # --- 1. NAVIGATION SETUP ---
        # BasicNavigator handles the high-level wheel movement
        self.navigator = BasicNavigator()

        # --- 2. LIDAR SETUP ---
        # Subscribe to /scan to find the closest object in front of the robot
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.closest_dist = 1.0  # Variable for object distance
        self.closest_angle = 0.0 # Variable for object angle

        # --- 3. MOVEIT SETUP ---
        # Connect to the MoveGroup action server to plan and move the arm
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

    def lidar_callback(self, msg):
        """Finds the closest object in a 180-degree front arc."""
        # 180° arc: Indices 0-90 (left-front) and 270-359 (right-front)
        front_indices = list(range(0, 91)) + list(range(270, 360))
        
        valid_hits = []
        for i in front_indices:
            dist = msg.ranges[i]
            # Filter: Ignore 0, infinity, and anything closer than 15cm (robot's own body)
            if 0.15 < dist < 1.5:
                angle = msg.angle_min + (i * msg.angle_increment)
                valid_hits.append((dist, angle))

        if valid_hits:
            # Sort the list by distance and pick the smallest one
            self.closest_dist, self.closest_angle = min(valid_hits, key=lambda x: x[0])

    def send_gripper_goal(self, close=True):
        """Sends a goal to the gripper group using the confirmed joint name."""
        goal_msg = MoveGroup.Goal()
        
        # Ensure this matches your MoveIt SRDF (usually 'gripper')
        goal_msg.request.group_name = "gripper"
        
        joint_con = JointConstraint()
        # Use the exact name from your /joint_states (the right is a mirror of the left)
        # We need to actually control the left
        joint_con.joint_name = "gripper_left_joint" 
        
        # Simulation Values for OpenManipulator-X:
        # Open: 0.019 (positive value)
        # Closed: -0.01 (negative value creates the clamping force)
        joint_con.position = -0.01 if close else 0.019
        
        joint_con.tolerance_above = 0.0005
        joint_con.tolerance_below = 0.0005
        joint_con.weight = 1.0

        constraints = Constraints()
        constraints.joint_constraints.append(joint_con)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(f"{'Closing' if close else 'Opening'} gripper...")
        
        # Check for the action server
        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveGroup action server not available!")
            return

        return self.move_group_client.send_goal_async(goal_msg)

    def send_pick_goal(self, offset=0.0):
        """Calculates a Cartesian goal and sends it to MoveIt."""

        # Calculation with Offset
        # reach_distance = LiDAR_Distance + Offset
        reach_dist = (self.closest_dist + offset)

        # 1. MATH: Convert LiDAR (r, theta) to Cartesian (x, y)
        x = reach_dist * math.cos(self.closest_angle)
        y = reach_dist * math.sin(self.closest_angle)
       
        
        # x = (self.closest_dist - 0.04) * math.cos(self.closest_angle)
        # y = (self.closest_dist - 0.04) * math.sin(self.closest_angle)
        z = 0.08  # Meters above arm base

        # 2. POSE: The target point
        target_p = Pose()
        target_p.position.x, target_p.position.y, target_p.position.z = x, y, z
        target_p.orientation.w = 1.0

        # 3. POSITION CONSTRAINT: Define the target link and tolerance
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "link1"        # Arm base
        pos_con.link_name = "end_effector_link"  # Standard TB3 link name
        
        box = SolidPrimitive()
        box.type, box.dimensions = SolidPrimitive.BOX, [0.02, 0.02, 0.02]
        
        pos_con.constraint_region.primitives.append(box)
        pos_con.constraint_region.primitive_poses.append(target_p)
        pos_con.weight = 1.0

        # 4. THE FIX: Create the Constraints object FIRST
        # You cannot access .position_constraints on a list!
        my_constraints = Constraints()
        my_constraints.position_constraints.append(pos_con)

        # 5. PACK ACTION GOAL
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm" 
        # Now append the fully built constraints object to the list
        goal_msg.request.goal_constraints.append(my_constraints)

        self.get_logger().info(f"Targeting Object at X: {x:.2f}, Y: {y:.2f}")
        
        # Ensure MoveIt is actually there before sending
        if not self.move_group_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("MoveGroup action server not available! Did you launch MoveIt?")
            return

        return self.move_group_client.send_goal_async(goal_msg)

    def run_mission(self):
        # I. INITIAL POSE (Tell the robot where it is on the map)
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.get_clock().now().to_msg()
        init_pose.pose.position.x, init_pose.pose.position.y, init_pose.pose.orientation.w = 0.5, 0.9, 1.0
        self.navigator.setInitialPose(init_pose)
        self.navigator.waitUntilNav2Active()

        # II. NAVIGATE ROUTE
        route = [[2.0, 0.7], [2.6, 0.7]]
        waypoints = []
        for p in route:
            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.pose.position.x, wp.pose.position.y, wp.pose.orientation.w = p[0], p[1], 1.0
            waypoints.append(deepcopy(wp))

        self.get_logger().info("Navigating waypoints...")
        self.navigator.followWaypoints(waypoints)

        # Spinning is critical so the LiDAR data updates while moving
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        # III. THE PICK 
        if self.navigator.getResult() == TaskResult.SUCCEEDED:
            self.get_logger().info("Arrived. Opening gripper...")
            self.send_gripper_goal(close=False) # Open first
            time.sleep(2.0)

            self.get_logger().info("Lining up arm...")
            self.send_pick_goal(offset=0.0)
            time.sleep(5.0) # Wait for reach to finish

            self.get_logger().info("Reaching for object...")
            self.send_pick_goal(offset=0.07)
            time.sleep(5.0) # Wait for reach to finish

            self.get_logger().info("Grasping...")
            self.send_gripper_goal(close=True) # Close claws
            time.sleep(2.0)

        # IV. RETURN HOME
        self.get_logger().info("Returning Home...")
        self.navigator.goToPose(init_pose)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    mission = TB3FinalMission()
    try:
        mission.run_mission()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
