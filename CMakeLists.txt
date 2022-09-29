cmake_minimum_required(VERSION 3.0.2)
project(mdds30_controller)

# Capture compiling warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)
catkin_install_python(PROGRAMS
  src/mdds30_controller.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

