add_executable(week6 main.cpp)
target_include_directories(week6 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week6 ${catkin_LIBRARIES})
add_dependencies(week6 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
