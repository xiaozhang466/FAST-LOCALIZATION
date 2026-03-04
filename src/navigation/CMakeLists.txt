cmake_minimum_required(VERSION 3.0.2)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  move_base
  map_server
  costmap_2d
  nav_core
  navfn
  dwa_local_planner
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/cmd_vel_filter.py
    scripts/map_odom_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
