cmake_minimum_required(VERSION 2.8.3)
project(ip_webcam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV REQUIRED)
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
)
include_directories(
  ${catkin_INCLUDE_DIRS}
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

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ip_webcam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(ip_webcam_node
  ${catkin_LIBRARIES}
)
target_link_libraries(ip_webcam_node
  ${OpenCV_LIBS}
)