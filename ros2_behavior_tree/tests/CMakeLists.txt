include_directories(include)

ament_add_gtest(test_ros2_behavior_tree
  test_async_wait.cpp
  test_first_result.cpp
  test_forever.cpp
  test_recovery.cpp
  test_repeat_until.cpp
  test_ros2_action_client.cpp
  test_ros2_service_client.cpp
  test_throttle_tick_count.cpp
)

ament_target_dependencies(test_ros2_behavior_tree ${dependencies})

target_link_libraries(test_ros2_behavior_tree ${library_name} ros2_behavior_tree_nodes)

add_library(custom_test_nodes SHARED src/test_node_registrar.cpp)
ament_target_dependencies(custom_test_nodes ${dependencies})
target_compile_definitions(custom_test_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS ${library_name} custom_test_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
