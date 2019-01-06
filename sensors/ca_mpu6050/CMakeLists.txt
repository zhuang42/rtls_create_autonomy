if( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm" )
  cmake_minimum_required(VERSION 2.8.3)
  project(ca_mpu6050)

  ## Find catkin macros and libraries
  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    rospy
    sensor_msgs
    std_msgs
    tf
  )

  catkin_python_setup()
  catkin_package()
  include_directories(${catkin_INCLUDE_DIRS})
else()
  message( STATUS "Avoid compiling ca_mpu6050 because it's not an ARM architecture" )
endif()
