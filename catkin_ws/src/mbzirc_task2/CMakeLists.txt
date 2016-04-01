cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_task2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  jackal_control
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mbzirc_task2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${Open_CV_INCLUDE_DIRS}
)

add_executable(matrix_transform src/matrix_transform.cpp)
add_executable(image_processor src/image_processor.cpp)

target_link_libraries(matrix_transform
  ${catkin_LIBRARIES}
)

target_link_libraries(image_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


catkin_install_python(PROGRAMS bin/model_inst
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})