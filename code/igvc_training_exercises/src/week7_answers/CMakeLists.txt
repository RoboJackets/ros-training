add_executable(week7_start_light_detector start_light_detector.cpp)
target_include_directories(week7_start_light_detector PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week7_start_light_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(week7_start_light_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week7_line_detector line_detector.cpp)
target_include_directories(week7_line_detector PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week7_line_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(week7_line_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
