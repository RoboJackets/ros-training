add_executable(week3 main.cpp)
target_include_directories(week3 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week3 ${catkin_LIBRARIES})
add_dependencies(week3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
