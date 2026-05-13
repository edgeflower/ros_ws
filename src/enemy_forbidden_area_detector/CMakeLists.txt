cmake_minimum_required(VERSION 3.16)
project(enemy_forbidden_area_detector)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== 查找依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(armor_interfaces REQUIRED)
find_package(pluginlib REQUIRED)

# ==================== 生成自定义消息 ====================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EnemyForbiddenArea.msg"
)

# ==================== 包含目录 ====================
include_directories(include)

# ==================== 核心库 ====================
add_library(${PROJECT_NAME}_lib
  src/geometry_utils.cpp
  src/marker_utils.cpp
  src/enemy_forbidden_area_detector.cpp
)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  armor_interfaces
)
target_link_libraries(${PROJECT_NAME}_lib yaml-cpp)

# 让库可以使用自己生成的消息头文件
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

# ==================== 检测节点 ====================
add_executable(enemy_forbidden_area_detector_node
  src/enemy_forbidden_area_detector_node.cpp
)

target_link_libraries(enemy_forbidden_area_detector_node
  ${PROJECT_NAME}_lib
  yaml-cpp
  "${cpp_typesupport_target}"
)

ament_target_dependencies(enemy_forbidden_area_detector_node
  rclcpp
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  armor_interfaces
)

# ==================== BehaviorTree 插件 ====================
add_library(bt_enemy_in_forbidden_area SHARED
  src/bt_enemy_in_forbidden_area.cpp
)

target_link_libraries(bt_enemy_in_forbidden_area
  "${cpp_typesupport_target}"
)

ament_target_dependencies(bt_enemy_in_forbidden_area
  rclcpp
  behaviortree_ros2
)

pluginlib_export_plugin_description_file(behaviortree_cpp bt_plugins.xml)

# ==================== 安装 ====================
install(TARGETS
  enemy_forbidden_area_detector_node
  ${PROJECT_NAME}_lib
  bt_enemy_in_forbidden_area
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
