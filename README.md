#Pre requisits

First ros2, turtlebot_ws and gazebo needs to be installed. Installation guidie can be found **[here](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)**.


Make sure the following is being sourced before starting. Copy them into the ~/.bashrc file to be loaded on cmd start.

`source /opt/ros/jazzy/setup.bash`
`source ~/turtlebot3_ws/install/setup.bash `
`export ROS_DOMAIN_ID=30 #TURTLEBOT3`
`export LDS_MODEL=LDS-02`
`export TURTLEBOT3_MODEL=waffle`

To build;

`rm -rf build/ install/ log/ #(optional for clean build)`
`cd ~/Expirement_1`
`cd ~/expirement_1 && colcon build --symlink-install`
`source ~/.bashrc`

To start turtlebot on rasberrypi run this:
`ros2 launch turtlebot3_bringup robot.launch.py`

To start the turblebot in gazebo the experiment 1 map only:
`ros2 launch simulation_pkg custom_world.launch.py`
C
To start turtlebot navigation with nav2 run:
`ros2 launch navigation_pkg navigation_sim.launch.py`

To run scripts run:
`ros2 run navigation_pkg test1.py`


To manually drive the robot use:
`ros2 run turtlebot3_teleop teleop_keyboard`
To manually generate map;
`ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True` #creates the map as you move around
`ros2 run nav2_map_server map_saver_cli -f my_map`#to save the map
`ros2 launch turtlebot3_bringup rviz2.launch.py` #Use RViz2 to visualize robot sensors and motion
once scan is done, nav2 can be run with (set the proper path to map file)
`ros2 launch nav2_bringup localization_launch.py map:=/home/$USER/my_map.yaml use_sim_time:=True`
`ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/$USER/my_map.yaml `

