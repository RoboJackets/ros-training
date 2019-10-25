add_executable(week7 main.cpp)
target_include_directories(week7 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week7 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(week7 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

