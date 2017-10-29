cmake_minimum_required(VERSION 2.8.3)
project(follower_2)

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

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${SDF_INCLUDE_DIRS})

add_executable(follower_2 src/follower.cpp)
add_dependencies(follower_2 hal_msgs_generate_messages_cpp)
target_link_libraries(follower_2 ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})

add_executable(tf_pub src/locator.cpp)
add_dependencies(tf_pub hal_msgs_generate_messages_cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES} ${PCL_INCLUDE_DIRS})
