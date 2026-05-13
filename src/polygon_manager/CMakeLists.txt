cmake_minimum_required(VERSION 3.16)
project(polygon_manager)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# C++ 标准设置（现代 C++17）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 设置包含目录
include_directories(include)

# ==================== 核心库 ====================

add_library(polygon_manager_lib
  src/geometry_utils.cpp
  src/marker_utils.cpp
  src/polygon_manager.cpp
)
set_target_properties(polygon_manager_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_target_dependencies(polygon_manager_lib
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(polygon_manager_lib yaml-cpp)

# ==================== ROS 节点 ====================

add_executable(polygon_manager_node
  src/polygon_node.cpp
)

target_link_libraries(polygon_manager_node
  polygon_manager_lib
  yaml-cpp
)

ament_target_dependencies(polygon_manager_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  std_srvs
)

# ==================== BehaviorTree 插件 ====================

add_library(bt_enemy_in_area SHARED
  src/bt_enemy_in_area.cpp
)

target_link_libraries(bt_enemy_in_area
  polygon_manager_lib
  yaml-cpp
)

ament_target_dependencies(bt_enemy_in_area
  rclcpp
  geometry_msgs
  behaviortree_ros2
)

# ==================== RViz 面板插件 ====================

qt5_wrap_cpp(PANEL_MOC_FILES
  include/polygon_manager/polygon_edit_panel.hpp
)

add_library(polygon_edit_panel SHARED
  src/polygon_edit_panel.cpp
  ${PANEL_MOC_FILES}
)

target_link_libraries(polygon_edit_panel
  Qt5::Widgets
)

ament_target_dependencies(polygon_edit_panel
  rclcpp
  rviz_common
  std_msgs
  std_srvs
)

pluginlib_export_plugin_description_file(rviz_common polygon_edit_plugin.xml)

# ==================== 安装 ====================

install(TARGETS
  polygon_manager_node
  polygon_manager_lib
  bt_enemy_in_area
  polygon_edit_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
