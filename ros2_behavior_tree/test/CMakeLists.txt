include_directories(include)

ament_add_gtest(test_ros2_behavior_tree
  test_async_wait.cpp
  test_forever.cpp
  test_recovery.cpp
  test_repeat_until.cpp
  test_ros2_action.cpp
  test_ros2_service.cpp
  test_throttle_tick_count.cpp
)

ament_target_dependencies(test_ros2_behavior_tree ${dependencies})

target_link_libraries(test_ros2_behavior_tree ${library_name})
