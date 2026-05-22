cmake_minimum_required(VERSION 3.8)
project(rm_keepout_costmap_layer)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(polygon_keepout_layer SHARED
  src/polygon_keepout_layer.cpp
)

target_include_directories(polygon_keepout_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(polygon_keepout_layer PUBLIC cxx_std_17)

ament_target_dependencies(polygon_keepout_layer PUBLIC
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  pluginlib
)

target_link_libraries(polygon_keepout_layer PUBLIC yaml-cpp)

pluginlib_export_plugin_description_file(nav2_costmap_2d polygon_keepout_layer.xml)

install(
  TARGETS polygon_keepout_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
