cmake_minimum_required(VERSION 3.0.2)
project(establishtf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES EstablishTF
#  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
 src/EstablishTfFromPose.cpp
 )
 
target_link_libraries(${PROJECT_NAME}_node 
${catkin_LIBRARIES}
)
