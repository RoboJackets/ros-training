add_executable(week2_publisher publisher.cpp)
target_include_directories(week2_publisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week2_publisher ${catkin_LIBRARIES})
add_dependencies(week2_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week2_subscriber subscriber.cpp)
target_include_directories(week2_subscriber PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week2_subscriber ${catkin_LIBRARIES})
add_dependencies(week2_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week2_loop_publisher exercises/loop_publisher.cpp)
target_include_directories(week2_loop_publisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week2_loop_publisher ${catkin_LIBRARIES})
add_dependencies(week2_loop_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week2_string_subscriber exercises/string_subscriber.cpp)
target_include_directories(week2_string_subscriber PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week2_string_subscriber ${catkin_LIBRARIES})
add_dependencies(week2_string_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(week2_even_publisher exercises/even_publisher.cpp)
target_include_directories(week2_even_publisher PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(week2_even_publisher ${catkin_LIBRARIES})
add_dependencies(week2_even_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
