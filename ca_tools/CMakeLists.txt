cmake_minimum_required(VERSION 2.8.3)
project(ca_tools)

find_package(catkin REQUIRED COMPONENTS
  ca_move_base
  geometry_msgs
  joy_teleop
  roslint
  rospy
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    joy_teleop
    rospy
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
        PATTERN ".svn" EXCLUDE
)

roslint_python()
