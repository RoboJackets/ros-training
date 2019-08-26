add_executable(week4 main.cpp)
target_include_directories(week4 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week4 ${catkin_LIBRARIES})
add_dependencies(week4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
