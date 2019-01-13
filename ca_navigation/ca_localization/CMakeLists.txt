cmake_minimum_required(VERSION 2.8.3)
project(ca_localization)

find_package(catkin REQUIRED COMPONENTS
  robot_localization
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
