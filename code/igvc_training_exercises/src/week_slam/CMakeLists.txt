find_package(SUITESPARSE)
find_package(G2O)

if (SUITESPARSE_FOUND AND G2O_FOUND)
  message(STATUS "Found suitesparse and g2o, compiling week_slam")
  find_package(Eigen3 3.3 REQUIRED)
  add_executable(week_slam
    main.cpp
    landmark_registration.cpp
    mapper.cpp
    slam_node.cpp
    barrel_ransac.cpp
    g2o/vertex/vertex_circle.cpp
    g2o/vertex/vertex_robot_state.cpp
    g2o/edge/edge_point_on_circle.cpp
    g2o/edge/edge_imu.cpp
    g2o/edge/edge_landmark.cpp
    slam_node_params.cpp)
  target_include_directories(week_slam PRIVATE ${catkin_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})
  target_link_libraries(week_slam ${catkin_LIBRARIES} Eigen3::Eigen ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})
  add_dependencies(week_slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else()
  message(STATUS "Couldn't find suitesparse and g2o, not compiling week_slam")
endif()

