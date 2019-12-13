cmake_minimum_required(VERSION 3.5)
project(ros2_behavior_tree_msgs)

find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  action_msgs
  builtin_interfaces
  geometry_msgs
  std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PrintMessage.action"
  "msg/Waypoint.msg"
  "msg/Lane.msg"
  DEPENDENCIES ${dependencies}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
