#!/usr/bin/env python3

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from launch.actions import TimerAction

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""


def main() -> None:
    # Initialize the ROS 2 Python client library
    rclpy.init()

    # Initializes the Navigator. This will wait until it can 
    # find the Nav2 Action Servers (like 'navigate_to_pose')
    navigator = BasicNavigator()

    # Our goal coordinates in meters. 
    # [0,0] is the origin point defined in 'my_map.yaml'.
    inspection_route = [
        [0.5, 0.3],
        [2.0, 1.0],
        [3.0, 0.3],
    ]

    # AMCL (the localization node) needs a starting guess.
    # This block mimics clicking '2D Pose Estimate' in RViz.
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map' # Coordinates are relative to the map
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.50
    initial_pose.pose.position.y = 0.90
    # Quaternions: z=0, w=1 means "facing forward/East" (0 degrees rotation).
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    # Tells the robot where it is starting. If this doesn't match 
    # the robot's actual physical position, navigation will fail.
    navigator.setInitialPose(initial_pose)

    # This line pauses the script until the Map Server is up, 
    # the Costmaps are loaded, and the Planner is ready to take commands.
    navigator.waitUntilNav2Active()

    # Send our route
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    for pt in inspection_route:
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        # We must copy the object, otherwise 'inspection_points' 
        # would just be a list of 3 identical references to the last point.
        inspection_points.append(deepcopy(inspection_pose))

    # 'followWaypoints' sends the whole list to the Nav2 Waypoint Follower.
    # The robot will move to point 1, stop briefly, then move to point 2, etc.
    # This returns immediately while the robot moves in the background.
    wpf_task = navigator.followWaypoints(inspection_points)

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstration
    i = 0
    # As long as the robot hasn't finished, crashed, or been canceled:
    while not navigator.isTaskComplete():
        i += 1
        # 'getFeedback' allows us to see what the robot is thinking/doing live.
        feedback = navigator.getFeedback()
        # Print progress every 5 iterations so the terminal stays readable.
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(inspection_points))
            )

    # Check if we actually arrived or if something went wrong.
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection of shelves complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection of shelving was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = navigator.getTaskError()
        print(f'Inspection of shelving failed!:{error_code}:{error_msg}')
        print('Returning to start...')

    # Update time and send a 'GoToPose' command (direct A-to-B navigation)
    # back to the very first coordinate we defined.
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    go_to_pose_task = navigator.goToPose(initial_pose)
    # Wait until the 'Return Home' task is finished.
    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()