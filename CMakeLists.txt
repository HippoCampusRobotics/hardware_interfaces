cmake_minimum_required(VERSION 3.0.2)
project(hardware_interfaces)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  dynamic_reconfigure
)

catkin_install_python(PROGRAMS 
  nodes/barometer_node
  nodes/camera_servo_node
  nodes/light_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()
generate_dynamic_reconfigure_options(
    config/Servo.cfg
    config/Light.cfg
)
catkin_package(
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


