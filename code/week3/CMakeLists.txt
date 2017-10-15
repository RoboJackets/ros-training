cmake_minimum_required(VERSION 2.8.3)
project(basic_driver)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs hal_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  )

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(driver src/driver.cpp)
target_link_libraries(driver ${catkin_LIBRARIES})
