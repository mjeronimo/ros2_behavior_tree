include_directories(include)

ament_add_gtest(test_ros2_behavior_tree_nodes
  test_async_wait.cpp
  test_first_result.cpp
  test_forever.cpp
  test_recovery.cpp
  test_repeat_until.cpp
  test_round_robin.cpp
  test_throttle_tick_count.cpp
)

ament_add_gtest(test_ros2_service_client
  test_ros2_service_client.cpp
)

ament_add_gtest(test_ros2_action_client
  test_ros2_action_client.cpp
)

ament_target_dependencies(test_ros2_behavior_tree_nodes ${dependencies})
ament_target_dependencies(test_ros2_service_client ${dependencies})
ament_target_dependencies(test_ros2_action_client ${dependencies})

target_link_libraries(test_ros2_behavior_tree_nodes ${library_name} ros2_behavior_tree_nodes)
target_link_libraries(test_ros2_service_client ${library_name} ros2_behavior_tree_nodes)
target_link_libraries(test_ros2_action_client ${library_name} ros2_behavior_tree_nodes)

add_library(custom_test_nodes SHARED src/test_node_registrar.cpp)
ament_target_dependencies(custom_test_nodes ${dependencies})
target_compile_definitions(custom_test_nodes PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS ${library_name} custom_test_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
