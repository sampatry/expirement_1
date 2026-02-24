#!/usr/bin/env python3
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ManipulatorController(Node):

    def __init__(self):
        super().__init__('manipulator_controller')

        self.arm_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd'
        )
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.arm_joint_positions = [0.0] * 4
        self.gripper_position = 0.0
        self.joint_received = False

    def joint_state_callback(self, msg):
        if set(self.arm_joint_names).issubset(set(msg.name)):
            for i, joint in enumerate(self.arm_joint_names):
                index = msg.name.index(joint)
                self.arm_joint_positions[i] = msg.position[index]
        self.joint_received = True

    def wait_for_joint_states(self):
        self.get_logger().info('Waiting for joint states...')
        while not self.joint_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info('Joint states received.')

    def move_arm(self, joint_angles, delay=2.0):
        """
        joint_angles: [joint1, joint2, joint3, joint4] in radians
        delay: time in seconds to wait for motion to complete
        """
        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 0
        msg.points.append(point)
        self.arm_publisher.publish(msg)
        self.get_logger().info(f'Arm moving to: {joint_angles}')
        time.sleep(delay)

    def move_gripper(self, position, max_effort=10.0):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f'Gripper moving to: {position}')
        time.sleep(1.0)


def main():
    rclpy.init()
    node = ManipulatorController()

    # Spin in background to receive joint states
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    node.wait_for_joint_states()

    # --- Your sequence of commands here ---
    node.move_arm([0.0, -1.0, 0.3, 0.7], delay=3.0)   # home position
    node.move_gripper(0.019)                             # open gripper
    node.move_arm([0.5, -0.5, 0.2, 0.5], delay=3.0)   # move to pick
    node.move_gripper(0.0)                              # close gripper
    node.move_arm([0.0, -1.0, 0.3, 0.7], delay=3.0)   # back to home

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()