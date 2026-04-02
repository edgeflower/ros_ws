cmake_minimum_required(VERSION 3.8)
project(rm_behavior_tree)


# 设置 clangd
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(OpenCV 4 REQUIRED)
ament_auto_find_build_dependencies()


#build

set(PACKAGE_INCLUDE_DEPENDS
  rm_decision_interfaces
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs
  behaviortree_cpp
  behaviortree_ros2
  )



# 定义宏函数 add_target_dependencies
function(add_target_dependencies target)
  target_include_directories(${target} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
  ament_target_dependencies(${target} ${package_INCLUDE_DEPENDS})
  target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
endfunction()

ament_auto_add_executable(rm_behavior_tree 
    src/rm_behavior_tree.cpp
)

ament_auto_add_executable(rm_behavior_tree_me 
	src/rm_behavior_tree_me.cpp
)
target_link_libraries(rm_behavior_tree_me
  spdlog::spdlog
  fmt::fmt  # <--- 重点是添加这一行
)
# action

ament_auto_add_library(send_my_goal SHARED
  plugins/action/send_my_goal.cpp)
add_target_dependencies(send_my_goal)

ament_auto_add_library(sub_all_robot_location SHARED
  plugins/action/sub_all_robot_location.cpp)
add_target_dependencies(sub_all_robot_location)

ament_auto_add_library(get_current_location SHARED
  plugins/action/get_current_location.cpp
)
add_target_dependencies(get_current_location)

ament_auto_add_library(get_location SHARED
  plugins/action/get_location.cpp
)
add_target_dependencies(get_location)

ament_auto_add_library(get_robot_location SHARED
  plugins/action/get_robot_location.cpp
)
add_target_dependencies(get_robot_location)

ament_auto_add_library(move_around SHARED
  plugins/action/move_around.cpp
)
add_target_dependencies(move_around)

ament_auto_add_library(print_message SHARED
  plugins/action/print_message.cpp
)
add_target_dependencies(print_message)

ament_auto_add_library(send_goal SHARED
  plugins/action/send_goal.cpp)
add_target_dependencies(send_goal)

ament_auto_add_library(set_posture SHARED
  plugins/action/set_posture.cpp)
add_target_dependencies(set_posture)

# Set Posture Xin (Simulation) Action Node
ament_auto_add_library(set_posture_xin SHARED
  plugins/action/set_posture_xin.cpp)
add_target_dependencies(set_posture_xin)

ament_auto_add_library(sub_all_robot_hp SHARED
  plugins/action/sub_all_robot_hp.cpp)
add_target_dependencies(sub_all_robot_hp)

ament_auto_add_library(sub_game_status SHARED
  plugins/action/sub_game_status.cpp)
add_target_dependencies(sub_game_status)

ament_auto_add_library(sub_robot_status SHARED
  plugins/action/sub_robot_status.cpp)
add_target_dependencies(sub_robot_status)

ament_auto_add_library(sub_robot_posture SHARED
  plugins/action/sub_robot_posture.cpp)
add_target_dependencies(sub_robot_posture)

ament_auto_add_library(sub_armors SHARED
  plugins/action/sub_armors.cpp)
add_target_dependencies(sub_armors)

ament_auto_add_library(robot_control SHARED
 plugins/action/robot_control.cpp)
add_target_dependencies(robot_control)

ament_auto_add_library(sub_decision_num SHARED
  plugins/action/sub_decision_num.cpp)
add_target_dependencies(sub_decision_num)

ament_auto_add_library(enemy_position_filter SHARED
  plugins/action/enemy_position_filter.cpp)
add_target_dependencies(enemy_position_filter)

ament_auto_add_library(armor_to_goal SHARED
  plugins/action/armor_to_goal.cpp)
add_target_dependencies(armor_to_goal)

# Hold Position Action Node
ament_auto_add_library(hold_position SHARED
  plugins/action/hold_position.cpp)
add_target_dependencies(hold_position)

# Abort Navigation Action Node
ament_auto_add_library(abort_navigation SHARED
  plugins/action/abort_navigation.cpp)
add_target_dependencies(abort_navigation)

# Wait Action Node
ament_auto_add_library(wait SHARED
  plugins/action/wait.cpp)
add_target_dependencies(wait)

# Bag Recorder Action Node
ament_auto_add_library(bag_recorder SHARED
  plugins/action/bag_recorder.cpp)
add_target_dependencies(bag_recorder)

# Goal Manager Action Node
ament_auto_add_library(goal_manager SHARED
  plugins/action/goal_manager.cpp)
add_target_dependencies(goal_manager)

# Set Goal Action Node
ament_auto_add_library(set_goal SHARED
  plugins/action/set_goal.cpp)
add_target_dependencies(set_goal)

# Calculate Angle Action Node
ament_auto_add_library(calculate_angle SHARED
  plugins/action/calculate_angle.cpp)
add_target_dependencies(calculate_angle)

# Cancel Navigation Action Node
ament_auto_add_library(cancel_navigation SHARED
  plugins/action/cancel_navigation.cpp)
add_target_dependencies(cancel_navigation)

# Sub CmdVel Action Node
ament_auto_add_library(sub_cmd_vel SHARED
  plugins/action/sub_cmd_vel.cpp)
add_target_dependencies(sub_cmd_vel)

# Map Processor Node (independent ROS2 node)
ament_auto_add_executable(map_processor_node
  src/map_processor_node.cpp)
add_target_dependencies(map_processor_node)
target_link_libraries(map_processor_node ${OpenCV_LIBS})

# condition

ament_auto_add_library(is_attacked SHARED
  plugins/condition/is_attacked.cpp)
add_target_dependencies(is_attacked)

ament_auto_add_library(is_friend_ok SHARED
  plugins/condition/is_friend_ok.cpp)
add_target_dependencies(is_friend_ok)

ament_auto_add_library(is_game_time SHARED
  plugins/condition/is_game_time.cpp)
add_target_dependencies(is_game_time)

ament_auto_add_library(is_outpost_ok SHARED
  plugins/condition/is_outpost_ok.cpp)
add_target_dependencies(is_outpost_ok)

ament_auto_add_library(is_status_ok SHARED
  plugins/condition/is_status_ok.cpp)
add_target_dependencies(is_status_ok)

ament_auto_add_library(is_detect_enenmy SHARED
 plugins/condition/is_detect_enenmy.cpp)
add_target_dependencies(is_detect_enenmy)

ament_auto_add_library(is_goal_reached SHARED
 plugins/condition/is_goal_reached.cpp)
add_target_dependencies(is_goal_reached)

ament_auto_add_library(should_reset_observation SHARED
 plugins/condition/should_reset_observation.cpp)
add_target_dependencies(should_reset_observation)

# Confidence Hysteresis Condition Node
ament_auto_add_library(confidence_hysteresis SHARED
 plugins/condition/confidence_hysteresis.cpp)
add_target_dependencies(confidence_hysteresis)

# Check Hysteresis State Condition Node
ament_auto_add_library(check_hysteresis_state SHARED
 plugins/condition/check_hysteresis_state.cpp)
add_target_dependencies(check_hysteresis_state)

# Is Hp Ok Condition Node
ament_auto_add_library(is_hp_ok SHARED
  plugins/condition/is_hp_ok.cpp)
add_target_dependencies(is_hp_ok)

# Is Stuck Condition Node
ament_auto_add_library(is_stuck SHARED
  plugins/condition/is_stuck.cpp)
add_target_dependencies(is_stuck)

# control
ament_auto_add_library(decision_switch SHARED
  plugins/control/decision_switch.cpp)
add_target_dependencies(decision_switch)

# decorator
ament_auto_add_library(rate_controller SHARED
  plugins/decorator/rate_controller.cpp)
add_target_dependencies(rate_controller)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
