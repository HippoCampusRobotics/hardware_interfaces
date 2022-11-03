cmake_minimum_required(VERSION 3.0.2)
project(hardware_interfaces)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  dynamic_reconfigure
  fav_msgs
)

catkin_python_setup()
generate_dynamic_reconfigure_options(
    cfg/Servo.cfg
    cfg/Light.cfg
    cfg/Barometer.cfg
)

catkin_install_python(PROGRAMS 
  nodes/barometer_node
  nodes/camera_servo_node
  nodes/imu_bridge_node
  nodes/light_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_executable(esc_commander src/esc_commander.cpp src/packet.cpp src/cobs.cpp)
target_link_libraries(esc_commander ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(esc_commander fav_msgs_generate_messages_cpp)


