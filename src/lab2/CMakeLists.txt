cmake_minimum_required(VERSION 3.0.2)
project(lab2)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
)

generate_messages(
  DEPENDENCIES
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
