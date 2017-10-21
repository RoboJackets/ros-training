cmake_minimum_required(VERSION 2.8.3)
project(hal_control)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  hal_msgs
  message_generation
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs hal_msgs
  )

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
   FILES_MATCHING PATTERN "*.yaml"
)

add_executable(hal_control node/hal_control.cpp)
target_link_libraries(hal_control ${catkin_LIBRARIES})
add_dependencies(hal_control hal_msgs_generate_messages_cpp)

catkin_install_python(
    PROGRAMS
    scripts/controller_spawner.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
    scripts/spawn_hal.sh
    scripts/controller_spawner.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
