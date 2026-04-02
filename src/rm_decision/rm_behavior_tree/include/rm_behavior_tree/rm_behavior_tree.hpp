#ifndef RM_BEHAVIOR_TREE__RM_BEHAVIOR_TREE_HPP_
#define RM_BEHAVIOR_TREE__RM_BEHAVIOR_TREE_HPP_

/**********************************************************************
 * 代码思路：
 * 1. 初始化ROS2系统并创建一个行为树节点，加载行为树XML文件路径参数。
 * 2. 创建行为树工厂并注册各个插件（包括消息更新、机器人控制、目标发送等插件）。
 * 3. 通过行为树工厂从XML文件创建行为树，并启动Groot2调试发布器（端口1667）。
 * 4. （本示例中采用集中管理器统一tick，所以内部定时器已注释掉）
 **********************************************************************/

#include <behaviortree_ros2/ros_node_params.hpp>   // ROS2节点参数封装接口
#include <rclcpp/rclcpp.hpp>                       // ROS2核心接口
#include <vector>
#include "behaviortree_cpp/bt_factory.h"           // 行为树工厂，用于创建和管理行为树
#include "behaviortree_cpp/loggers/groot2_publisher.h" // Groot2调试发布器
#include "rm_behavior_tree/bt_conversions.hpp"     // 行为树类型转换（PoseStamped 等）
#include "behaviortree_cpp/utils/shared_library.h" // 共享库操作接口
#include "behaviortree_ros2/plugins.hpp"           // ROS2插件注册接口

namespace rm_behavior_tree {

class BehaviorTreeNode : public rclcpp::Node
{
public:
    BehaviorTreeNode() : rclcpp::Node("rm_behavior_tree")
    {
        declare_parameter<std::string>("style", "./3V3/rm_behavior_tree/rm_behavior_tree.xml");

        get_parameter_or<std::string>("style", bt_xml_path, "./3V3/rm_behavior_tree/config/attack_left.xml");

        // 输出当前加载的行为树XML路径到日志
        RCLCPP_INFO(this->get_logger(), "Load bt_xml:\e[1;42m %s \e[0m", bt_xml_path.c_str());

        // 创建行为树工厂，用于注册插件和生成行为树
        factory = std::make_shared<BT::BehaviorTreeFactory>();

        // 注意：不能在构造函数中调用 shared_from_this()
        // 需要在对象被 shared_ptr 管理后调用 init()
    }

    // 初始化方法：注册插件并创建行为树
    // 必须在对象被 shared_ptr 管理后调用
    void init()
    {
        // 注册所有插件
        registerPlugins();

        // 通过行为树工厂从XML文件创建行为树
        auto instance = factory->createTreeFromFile(bt_xml_path);
        tree_ptr = std::make_shared<BT::Tree>(std::move(instance));

        // 启动Groot2调试发布器，用于实时监控行为树状态，指定端口为1667
        const unsigned port = 1667;
        publisher = std::make_shared<BT::Groot2Publisher>(*tree_ptr, port);
    }

    // 提供一个public接口，返回内部行为树对象指针
    std::shared_ptr<BT::Tree> getTreePtr() { return tree_ptr; }

    // 获取指向自身的 shared_ptr (用于传递给插件)
    std::shared_ptr<BehaviorTreeNode> getSharedPtr()
    {
        // rclcpp::Node 继承了 enable_shared_from_this
        // 使用 static_pointer_cast 进行类型转换
        return std::static_pointer_cast<BehaviorTreeNode>(shared_from_this());
    }

private:
    // 注册所有插件
    void registerPlugins()
    {
        // Get shared node (this) to pass to plugins
        BT::RosNodeParams params_shared;
        params_shared.nh = getSharedPtr();

        BT::RosNodeParams params_update_msg;
        params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

        BT::RosNodeParams params_robot_control;
        params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
        params_robot_control.default_port_value = "robot_control";

        BT::RosNodeParams params_send_goal;
        params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
        params_send_goal.default_port_value = "goal_pose";

        BT::RosNodeParams params_send_my_goal;
        params_send_my_goal.nh = std::make_shared<rclcpp::Node>("send_my_goal");
        params_send_my_goal.default_port_value = "my_goal_pose";

        BT::RosNodeParams params_set_posture;
        params_set_posture.nh = std::make_shared<rclcpp::Node>("set_posture");
        params_set_posture.default_port_value = "set_sentry_posture";

        // Parameters for goal_manager (subscribes to /observation_points)
        BT::RosNodeParams params_goal_manager;
        params_goal_manager.nh = getSharedPtr();
        params_goal_manager.default_port_value = "/observation_points";

        // Parameters for should_reset_observation (subscribes to /observation_points)
        BT::RosNodeParams params_should_reset;
        params_should_reset.nh = getSharedPtr();
        params_should_reset.default_port_value = "/observation_points";

        // Parameters for get_robot_location (subscribes to /odometry)
        BT::RosNodeParams params_get_robot_location;
        params_get_robot_location.nh = getSharedPtr();
        params_get_robot_location.default_port_value = "/odometry";

        // Parameters for get_location (subscribes to /odometry)
        BT::RosNodeParams params_get_location;
        params_get_location.nh = getSharedPtr();
        params_get_location.default_port_value = "/odometry";

        // Parameters for cancel_navigation (uses shared node, subscribes to /action_name)
        BT::RosNodeParams params_cancel_navigation;
        params_cancel_navigation.nh = getSharedPtr();
        params_cancel_navigation.default_port_value = "/action_name";

        // Parameters for is_goal_reached (subscribes to /goal_pose and /robot_location)
        BT::RosNodeParams params_calulate_angle;
        params_calulate_angle.nh = getSharedPtr();
        params_calulate_angle.default_port_value = "/goal_pose"; // 主要输入，另外还会订阅 /robot_location


        const std::vector<std::string> msg_update_plugins_libs = {
            "sub_all_robot_hp",
            "sub_robot_status",
            "sub_game_status",
            "sub_armors",
            "sub_decision_num",
            "sub_all_robot_location",
            "sub_robot_posture",
        };

        const std::vector<std::string> bt_plugins_libs = {
            "rate_controller",
            "decision_switch",
            "is_game_time",
            "is_status_ok",
            "is_detect_enenmy",
            "is_attacked",
            "is_friend_ok",
            "is_outpost_ok",
            "is_hp_ok",
            "get_current_location",
            "move_around",
            "print_message",
            "enemy_position_filter",
            "armor_to_goal",
            "confidence_hysteresis",
            "hold_position",
            "check_hysteresis_state",
            "abort_navigation",
            "wait",
            "is_goal_reached",
            "set_posture_xin",
            "bag_recorder"
            // Note: goal_manager, should_reset_observation, get_robot_location removed
            // They will be registered using RegisterRosNode below
        };

        // 注册消息更新插件
        for (const auto & plugin : msg_update_plugins_libs) {
            RegisterRosNode(*factory, BT::SharedLibrary::getOSName(plugin), params_update_msg);
        }

        // 注册行为树功能插件
        for (const auto & plugin : bt_plugins_libs) {
            factory->registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
        }

        // 注册目标发送和机器人控制插件
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("send_goal"), params_send_goal);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("send_my_goal"), params_send_my_goal);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("set_posture"), params_set_posture);

        // 注册观察点相关插件（使用共享节点）
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("goal_manager"), params_goal_manager);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("should_reset_observation"), params_should_reset);
        // is_goal_reached: moved to bt_plugins_libs (uses Blackboard, not RosNodeParams)
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("get_robot_location"), params_get_robot_location);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("get_location"), params_get_location);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("cancel_navigation"), params_cancel_navigation);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("calculate_angle"), params_calulate_angle);
    }

    // 成员变量
    std::string bt_xml_path;                                    // 行为树XML文件路径
    std::shared_ptr<BT::BehaviorTreeFactory> factory;           // 行为树工厂
    std::shared_ptr<BT::Groot2Publisher> publisher;             // Groot2调试发布器
    std::shared_ptr<BT::Tree> tree_ptr;                         // 行为树对象指针
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__RM_BEHAVIOR_TREE_HPP_