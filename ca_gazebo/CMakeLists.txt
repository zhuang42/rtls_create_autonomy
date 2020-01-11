cmake_minimum_required(VERSION 2.8.3)
project(ca_gazebo)

add_compile_options(-std=c++11)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	ca_msgs
	gazebo_plugins
	gazebo_ros
	roscpp
	roslint
	tf2
	tf2_ros
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES
		create_bumper_plugin
		model_pose_publisher_plugin
)

include_directories(include
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

add_library(create_bumper_plugin src/create_bumper_plugin.cpp)
add_dependencies(create_bumper_plugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(create_bumper_plugin
	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(model_pose_publisher_plugin src/model_pose_publisher_plugin.cpp)
target_link_libraries(model_pose_publisher_plugin
	${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch models worlds
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
		create_bumper_plugin
		model_pose_publisher_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++11")
roslint_cpp()
