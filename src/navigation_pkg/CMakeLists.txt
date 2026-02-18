cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/multi_waypoint.py
  scripts/single_goal_nav.py
  scripts/test1.py
  DESTINATION lib/${PROJECT_NAME})


# Install launch and config files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

# Python packaging support
ament_python_install_package(${PROJECT_NAME})

ament_package()