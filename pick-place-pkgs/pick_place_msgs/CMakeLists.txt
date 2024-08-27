cmake_minimum_required(VERSION 2.8.3)
project(pick_place_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  geometry_msgs
  genmsg
)

add_service_files(
  FILES
  DetectBottles.srv
  MoveArm.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES duckietown_msgs
   CATKIN_DEPENDS
   rospy
   std_msgs
   geometry_msgs
   message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
