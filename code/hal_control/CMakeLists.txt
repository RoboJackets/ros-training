cmake_minimum_required(VERSION 2.8.3)
project(hal_control)
find_package(catkin REQUIRED)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package()

find_package(catkin REQUIRED COMPONENTS
	rospy
	roscpp
	std_msgs
	hal_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

add_executable(hal_control node/hal_control.cpp)
target_link_libraries(hal_control ${catkin_LIBRARIES})

catkin_install_python(
		PROGRAMS
		scripts/controller_spawner.py
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
		scripts/spawn_hal.sh
		scripts/controller_spawner.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
