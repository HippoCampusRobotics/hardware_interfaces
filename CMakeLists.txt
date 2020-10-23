cmake_minimum_required(VERSION 2.8.3)
project(hardware_interfaces)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
)

catkin_python_setup()
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
