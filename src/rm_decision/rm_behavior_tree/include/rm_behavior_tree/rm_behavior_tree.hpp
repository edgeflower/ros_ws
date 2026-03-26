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
#include "behaviortree_cpp/utils/shared_library.h" // 共享库操作接口
#include "behaviortree_ros2/plugins.hpp"           // ROS2插件注册接口

namespace rm_behavior_tree {

class BehaviorTreeNode : public rclcpp::Node {
public:
    BehaviorTreeNode() : rclcpp::Node("rm_behavior_tree")
    {
        declare_parameter<std::string>("style", "./3V3/rm_behavior_tree/rm_behavior_tree.xml");

        get_parameter_or<std::string>("style", bt_xml_path, "./3V3/rm_behavior_tree/config/attack_left.xml");

        // 输出当前加载的行为树XML路径到日志
        RCLCPP_INFO(this->get_logger(), "Load bt_xml:\e[1;42m %s \e[0m", bt_xml_path.c_str());

        // 创建行为树工厂，用于注册插件和生成行为树
        factory = std::make_shared<BT::BehaviorTreeFactory>();

        // 注册所有插件，包括消息更新插件、机器人控制插件、目标发送插件等
        registerPlugins();

        // 通过行为树工厂从XML文件创建行为树
        tree = factory->createTreeFromFile(bt_xml_path);

        // 启动Groot2调试发布器，用于实时监控行为树状态，指定端口为1667
        const unsigned port = 1667;
        publisher = std::make_shared<BT::Groot2Publisher>(tree, port);

        //【如果使用集中管理器统一tick，则不需要内部定时器】
//        timer_ = this->create_wall_timer(
//            std::chrono::milliseconds(50),
//            [this]() { this->onTimer(); }
//        );
    }

    // 提供一个public接口，返回内部行为树对象引用
    BT::Tree& getTree() { return tree; }

private:
    // 注册所有插件
    void registerPlugins() {
        BT::RosNodeParams params_update_msg;
        params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

        BT::RosNodeParams params_robot_control;
        params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
        params_robot_control.default_port_value = "robot_control";

        BT::RosNodeParams params_send_goal;
        params_send_goal.nh = std::make_shared<rclcpp::Node>("send_goal");
        params_send_goal.default_port_value = "goal_pose";

        BT::RosNodeParams params_set_posture;
        params_set_posture.nh = std::make_shared<rclcpp::Node>("set_posture");
        params_set_posture.default_port_value = "set_posture";


        const std::vector<std::string> msg_update_plugins_libs = {
            "sub_all_robot_hp",
            "sub_robot_status",
            "sub_game_status",
            "sub_armors",
            "sub_decision_num",
            "sub_all_robot_location",
            "get_location",
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
            "get_current_location",
            "get_robot_location",
            "move_around",
            "print_message",
            "pick_best_armor",
            "armor_to_goal",
            "set_posture",
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
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);
        RegisterRosNode(*factory, BT::SharedLibrary::getOSName("set_posture"), params_set_posture);
    }

    // 定时器回调（不再使用，由管理器统一tick）
//    void onTimer() {
//        tree.tickWhileRunning(std::chrono::milliseconds(10));
//    }

    // 成员变量
    std::string bt_xml_path;                                    // 行为树XML文件路径
    std::shared_ptr<BT::BehaviorTreeFactory> factory;           // 行为树工厂
    std::shared_ptr<BT::Groot2Publisher> publisher;             // Groot2调试发布器
    // std::shared_ptr<rclcpp::TimerBase> timer_;                // 定时器（此处不再使用）
    BT::Tree tree;                                            // 行为树对象
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__RM_BEHAVIOR_TREE_HPP_
