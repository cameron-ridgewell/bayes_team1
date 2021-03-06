cmake_minimum_required(VERSION 2.8.3)
project(ip_webcam)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  cmake_modules
  tf
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ip_webcam
#  CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${Open_CV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(ip_webcam
#   src/${PROJECT_NAME}/ip_webcam.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(ip_webcam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(ip_webcam_node src/ip_webcam_node.cpp)
add_executable(object_tracker src/object_tracker.cpp)
add_executable(kalman_processor src/kalman_processor.cpp)
add_executable(kalman_view src/kalman_view.cpp)
add_executable(odom_manager src/odom_manager.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ip_webcam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ip_webcam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(object_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(kalman_processor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(kalman_view
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(odom_manager
  ${catkin_LIBRARIES}
)