add_executable(week5 main.cpp)
target_include_directories(week5 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week5 ${catkin_LIBRARIES})
add_dependencies(week5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
