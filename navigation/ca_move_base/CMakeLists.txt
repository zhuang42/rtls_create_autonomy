cmake_minimum_required(VERSION 2.8.3)
project(ca_move_base)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  amcl
  iris_lama
  iris_lama_ros
  move_base
  move_base_msgs
  octomap_server
  roscpp
  tf
)

catkin_package(
 CATKIN_DEPENDS
  move_base_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(send_move_base_goal src/send_robot_goal.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(send_move_base_goal
  ${catkin_LIBRARIES}
)

install(TARGETS send_move_base_goal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
