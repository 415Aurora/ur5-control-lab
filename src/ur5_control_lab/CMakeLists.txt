cmake_minimum_required(VERSION 3.0.2)
project(ur5_control_lab)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  kdl_parser_py
  tf2_ros
  tf_conversions
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS actionlib control_msgs kdl_parser_py rospy sensor_msgs std_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
