cmake_minimum_required(VERSION 2.8.3)
project(node_example)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
