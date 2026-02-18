cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(turtlebot3_description REQUIRED)

install(DIRECTORY launch urdf worlds models
  DESTINATION share/${PROJECT_NAME})


install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
