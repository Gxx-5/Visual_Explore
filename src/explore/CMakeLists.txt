cmake_minimum_required(VERSION 3.0.2)
project(explore)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -std=c++11 -pthread -g -march=native")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Boost REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES explore
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
${PROJECT_SOURCE_DIR}/include
${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
src/CalCost.cc
src/Costcube.cc
src/KDTree.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
  ${PCL_LIBRARIES}
)


