add_executable(week_scanmatching
  main.cpp
  icp/icp.cpp
  scanmatching_localizer.cpp)
target_include_directories(week_scanmatching PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week_scanmatching ${catkin_LIBRARIES})
add_dependencies(week_scanmatching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
