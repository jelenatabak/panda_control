cmake_minimum_required(VERSION 2.8.3)
project(panda_control)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  tf
  message_generation
  impedance_control
)

catkin_python_setup()

add_service_files(DIRECTORY srv FILES GraspSrv.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})





