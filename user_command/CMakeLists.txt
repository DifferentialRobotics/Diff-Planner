cmake_minimum_required(VERSION 3.0.2)
project(multipoint)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS EIGEN3
  INCLUDE_DIRS
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  
)

add_executable(multipointplan 
  src/multipointplan.cpp)

target_link_libraries(multipointplan
  yaml-cpp
  ${catkin_LIBRARIES}
)

