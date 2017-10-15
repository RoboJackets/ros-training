cmake_minimum_required(VERSION 2.8.3)
project(basic_driver)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(driver src/driver.cpp)
target_link_libraries(driver ${catkin_LIBRARIES})
