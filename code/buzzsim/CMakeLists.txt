cmake_minimum_required(VERSION 3.9.0)
project(igvc_buzzsim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-W -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  geometry_msgs
  std_msgs
  turtlesim
  std_srvs)

find_package(CGAL REQUIRED COMPONENTS Core)
set(CGAL_LIBRARIES CGAL::CGAL CGAL::CGAL_Core)

find_package(Yaml-cpp REQUIRED)

###########################
#       QT stuff          #
###########################
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(QT_LIBRARIES Qt5::Widgets)

###################################
## catkin specific configuration ##
###################################

catkin_package(CATKIN_DEPENDS
  roslib
  pcl_ros
  geometry_msgs
  message_runtime
  std_msgs
  std_srvs)

###########
## Build ##
###########
set(buzzsim_SRCS
  src/buzzsim/main.cpp
  src/buzzsim/app.cpp
  src/buzzsim/frame.cpp
  src/buzzsim/turtle.cpp
  src/buzzsim/motion.cpp
  src/buzzsim/world.cpp
  src/buzzsim/world_config_parser.cpp
  src/buzzsim/obstacle.cpp
  src/buzzsim/obstacle_parser.cpp
  src/buzzsim/sensors/lidar.cpp
  src/buzzsim/sensors/print_utils.cpp
  src/buzzsim/sensors/lidar_painter.cpp
  )
set(buzzsim_HDRS
  include/buzzsim/app.h
  include/buzzsim/frame.h
  )

qt5_wrap_cpp(buzzsim_MOCS ${buzzsim_HDRS})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(buzzsim ${buzzsim_SRCS} ${buzzsim_MOCS})
target_include_directories(buzzsim PRIVATE include ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
target_link_libraries(buzzsim ${QT_LIBRARIES} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${CGAL_LIBRARIES})
add_dependencies(buzzsim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
