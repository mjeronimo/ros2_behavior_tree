cmake_minimum_required(VERSION 3.5)
project(ros2_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_behavior_tree_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree.cpp
)

add_library(ros2_behavior_tree_nodes SHARED
  src/node_registrar.cpp
)

add_library(example_custom_nodes SHARED
  examples/custom_nodes/sample_bt_node_registrar.cpp
)

add_executable(minimal
  examples/minimal/main.cpp
)

add_executable(node
  examples/node/main.cpp
  examples/node/sample_node.cpp
)

add_executable(custom_nodes
  examples/custom_nodes/main.cpp
  examples/custom_nodes/sample_node.cpp
)

add_executable(action_server_node
  examples/action_server_node/main.cpp
  examples/action_server_node/sample_action_server_node.cpp
)

add_executable(lifecycle_node
  examples/lifecycle_node/main.cpp
  examples/lifecycle_node/sample_lifecycle_node.cpp
)

add_executable(action_server_lifecycle_node
  examples/action_server_lifecycle_node/main.cpp
  examples/action_server_lifecycle_node/sample_action_server_lifecycle_node.cpp
)

set(dependencies
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  ros2_behavior_tree_msgs
  std_msgs
  std_srvs
  example_interfaces
)

ament_target_dependencies(${library_name} ${dependencies})
ament_target_dependencies(ros2_behavior_tree_nodes ${dependencies})
ament_target_dependencies(example_custom_nodes ${dependencies})
ament_target_dependencies(minimal ${dependencies})
ament_target_dependencies(node ${dependencies})
ament_target_dependencies(custom_nodes ${dependencies})
ament_target_dependencies(action_server_node ${dependencies})
ament_target_dependencies(lifecycle_node ${dependencies})
ament_target_dependencies(action_server_lifecycle_node ${dependencies})

target_link_libraries(minimal ${library_name})
target_link_libraries(node ${library_name})
target_link_libraries(custom_nodes ${library_name})
target_link_libraries(action_server_node ${library_name})
target_link_libraries(lifecycle_node ${library_name})
target_link_libraries(action_server_lifecycle_node ${library_name})

target_compile_definitions(ros2_behavior_tree_nodes PRIVATE
  BT_PLUGIN_EXPORT
)

target_compile_definitions(example_custom_nodes PRIVATE
  BT_PLUGIN_EXPORT
)

install(TARGETS ${library_name} ros2_behavior_tree_nodes example_custom_nodes minimal node custom_nodes action_server_node lifecycle_node action_server_lifecycle_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories(include)

ament_export_libraries(
  ${library_name}
  ros2_behavior_tree_nodes
)

ament_export_dependencies(${dependencies})

ament_package()
