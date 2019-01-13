if( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" )
  message( STATUS "Avoid compiling ca_localization because it's an ARM architecture" )
else()
  cmake_minimum_required(VERSION 2.8.3)
  project(ca_localization)

  find_package(catkin REQUIRED COMPONENTS
    robot_localization
  )

  catkin_package()

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif()
