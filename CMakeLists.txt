cmake_minimum_required(VERSION 2.8.3)
project(hardware_interfaces)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  camera_info_manager
  dynamic_reconfigure
)

find_package(OpenCV)

FIND_LIBRARY(ARDUCAM_LIBRARY arducam_mipicamera /usr/lib/)

catkin_python_setup()
generate_dynamic_reconfigure_options(
  cfg/Camera.cfg
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 image_transport
                 cv_bridge
                 camera_info_manager
                 dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ov9281_info src/ov9281_info.cpp)
target_link_libraries(ov9281_info
  ${catkin_LIBRARIES}
  ${ARDUCAM_LIBRARY}
)

add_executable(camera_node src/camera_node.cpp src/camera_ov9281.cpp)
target_link_libraries(camera_node
  ${catkin_LIBRARIES}
  ${ARDUCAM_LIBRARY}
  ${OpenCV_LIBRARIES}
)
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

