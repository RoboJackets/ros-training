cmake_minimum_required(VERSION 2.8.3)
project(follower)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hal_msgs
  message_generation
  pcl_ros
  laser_geometry
  tf)

find_package(PCL REQUIRED filters)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs hal_msgs
  )

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(follower src/follower.cpp)
add_dependencies(follower hal_msgs_generate_messages_cpp)
target_link_libraries(follower ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

add_executable(converter src/converter.cpp)
add_dependencies(converter hal_msgs_generate_messages_cpp)
target_link_libraries(converter ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
