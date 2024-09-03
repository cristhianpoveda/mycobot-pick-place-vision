cmake_minimum_required(VERSION 2.8.3)
project(pick_place_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  std_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
)

add_service_files(
  FILES
  DetectBottles.srv
  VerifyPicking.srv
  SendCoords.srv
  SendCoord.srv
  SendAngles.srv
  SendAngle.srv
  GetJoints.srv
)

add_action_files(
  DIRECTORY action 
  FILES
  PickPlace.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES duckietown_msgs
   CATKIN_DEPENDS
   rospy
   std_msgs
   geometry_msgs
   message_runtime
   actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
