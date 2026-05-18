cmake_minimum_required(VERSION 3.16)
project(robot_area_detector)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rm_decision_interfaces REQUIRED)

# 自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotAreaStatus.msg"
  DEPENDENCIES std_msgs
)

# Composable 节点库
add_library(robot_area_detector_component SHARED
  src/robot_area_detector_node.cpp
)

ament_target_dependencies(robot_area_detector_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rm_decision_interfaces
)

target_link_libraries(robot_area_detector_component yaml-cpp)

# 链接本包生成的消息 typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robot_area_detector_component "${cpp_typesupport_target}")

rclcpp_components_register_node(robot_area_detector_component
  PLUGIN "robot_area_detector::RobotAreaDetectorNode"
  EXECUTABLE robot_area_detector_node
)

# 安装
install(TARGETS robot_area_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS robot_area_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
