cmake_minimum_required(VERSION 3.16)
project(robot_area_detector)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotAreaStatus.msg"
  DEPENDENCIES std_msgs
)

# 节点
add_executable(robot_area_detector_node
  src/robot_area_detector_node.cpp
)

ament_target_dependencies(robot_area_detector_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(robot_area_detector_node yaml-cpp)

# 链接本包生成的消息 typesupport
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(robot_area_detector_node "${cpp_typesupport_target}")

# 安装
install(TARGETS robot_area_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
