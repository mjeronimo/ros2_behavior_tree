cmake_minimum_required(VERSION 3.5)
project(ros2_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
  src/recovery_node.cpp
)

add_library(ros2_behavior_tree_nodes SHARED
  src/bt_nodes.cpp
)

add_executable(simple examples/simple/main.cpp)

set(dependencies
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  std_srvs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

ament_target_dependencies(ros2_behavior_tree_nodes
  ${dependencies}
)

ament_target_dependencies(simple
  ${dependencies}
)

target_link_libraries(simple ${library_name})

target_compile_definitions(ros2_behavior_tree_nodes PRIVATE
  BT_PLUGIN_EXPORT
)

install(TARGETS ${library_name} ros2_behavior_tree_nodes simple
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
endif()

ament_export_include_directories(include)

ament_export_libraries(
  ${library_name}
  ros2_behavior_tree_nodes
)

ament_export_dependencies(${dependencies})

ament_package()
