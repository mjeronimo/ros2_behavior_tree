cmake_minimum_required(VERSION 3.5)
project(ros2_behavior_tree)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
  src/recovery_node.cpp
)

set(dependencies
  behaviortree_cpp_v3
  geometry_msgs
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rosidl_default_generators
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(bt_executor
  examples/bt_executor/bt_executor.cpp
  examples/bt_executor/main.cpp
)

ament_target_dependencies(bt_executor
  ${dependencies}
)

target_link_libraries(bt_executor
  ${library_name} 
)

install(TARGETS ${library_name} bt_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
#  This package publishes messages from tf2_geometry_msgs
ament_export_dependencies(tf2_geometry_msgs)

ament_package()
