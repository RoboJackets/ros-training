cmake_minimum_required(VERSION 3.9.0)
project(igvc_training_exercises)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-W -Wall -Wextra)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf_conversions
  std_msgs
  igvc_buzzsim)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  roscpp
  tf
  tf_conversions
  std_msgs
  igvc_buzzsim)

include_directories(include)

add_subdirectory(src/week2)
add_subdirectory(src/week3)
add_subdirectory(src/week4)
add_subdirectory(src/week5)
add_subdirectory(src/week6)

add_subdirectory(src/week_scanmatching)
add_subdirectory(src/week_slam)
