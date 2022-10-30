cmake_minimum_required(VERSION 2.8.3)
project(lane_control)


find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
)

catkin_python_setup()


generate_dynamic_reconfigure_options(
  cfg/Config.cfg
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
