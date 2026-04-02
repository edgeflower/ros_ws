#include <behaviortree_ros2/ros_node_params.hpp>
#include <ostream>
#include <rclcpp/node.hpp>
#include <vector>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    std::string bt_xml_path;
    auto node = std::make_shared<rclcpp::Node>("rm_behavior_tree");

    //! 这个 rm_behavior_tree.xml 还没写
    node->declare_parameter<std::string>(
        "style" , "./3V3/rm_nav/rm_behavior_tree/config/test.xml"); 
    
    //! 这个也没写          attack_left.xml
    node->get_parameter_or<std::string>(
        "style", "./3V3/rm_nav/rm_behavior_tree/config/test.xml"
    );

    // 输出程序启动提示及加载的XML文件路径
    std::cout << "Start RM_Behavior_tree" << std::endl;
    RCLCPP_INFO(node->get_logger(),"load bt_xml: \033[1;42m %s \033[0m", bt_xml_path.c_str());

    //------------------定义ROS节点参数对象---------------------------
    // 为不同插件注册对应的 ROS 节点封装对象

    // （1） 用于消息更新插件： 这些插件负责订阅机器人血量、状态、比赛状态等信息
    BT::RosNodeParams params_update_msg;
    params_update_msg.nh = std::make_shared<rclcpp::Node>("update_msg");

    // (2) 用于机器人控制插件：发送控制指令给机器人
    BT::RosNodeParams params_robot_control;
    params_robot_control.nh = std::make_shared<rclcpp::Node>("robot_control");
    // 设置该插件节点的默认端口名称，便于在行为行为树中传递命令
    params_robot_control.default_port_value = "robot_control";

    // (3) 用于发送目标位置信息的插件： 向机器人发送目标点数据
    BT::RosNodeParams params_send_goal;
    params_send_goal.nh = std::make_shared<rclcpp::Node>("goal_pose");
    // 设置该插件节点的默认端口名称 "goal_pose"，用于传递目标位置信息
    params_send_goal.default_port_value = "goal_pose";

    // (4) 用于获取机器人位置信息的插件： 从里程计获取机器人当前位置
    BT::RosNodeParams params_get_location;
    params_get_location.nh = std::make_shared<rclcpp::Node>("get_location");
    // 设置默认topic名称为lidar_odometry，与XML配置保持一致
    params_get_location.default_port_value = "/lidar_odometry";

    //-------------------插件库名称列表----------------------------
    // 列出所有需要注册的插件名称，每个插件均封装成一个共享库文件

    // 消息更新插件：用于订阅各类比赛和状态信息
    const std::vector<std::string> msg_update_plugins_libs = {
        "sub_all_robot_hp",
        "sub_robot_status",
        "sub_game_status",
        "sub_armors",
        "sub_decision_num",
        
    };

    // 行为树功能插件： 实现实际的决策、状态判断和动作执行
    const std::vector<std::string> bt_plugins_libs {
        "rate_controller",
        "decision_switch",
        "is_game_time",
        "is_status_ok",
        "is_detect_enenmy",
        "is_attacked",
        "is_friend_ok",
        "is_outpost_ok",
        "get_current_location",
        "move_around",
        "print_message",
        "bag_recorder",
    };
    //--------------------------注册插件--------------------------
    // 依次将各插件注册到行为树厂中，便于后续再行为树中调用

    // 注册消息更新插件注册并绑定ROS节点参数

    for (const auto & plugin : msg_update_plugins_libs){
        // 通过 BT::SharedLibrary::getOSName 转换插件名称为对应操作系统的共享库名称
        // RegisterRosNode 函数实现插件注册并绑定 ROS节点参数
        RegisterRosNode(factory, BT::SharedLibrary::getOSName(plugin), params_update_msg);
    }

    // 注册行为树插件，直接加载共享库中的节点实现

    for(const auto & plugin : bt_plugins_libs){
        factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }

    RegisterRosNode(factory, BT::SharedLibrary::getOSName("send_goal"),params_send_goal);

    RegisterRosNode(factory, BT::SharedLibrary::getOSName("get_location"), params_get_location);

    RegisterRosNode(factory, BT::SharedLibrary::getOSName("robot_control"), params_robot_control);
    //--------------------------创建行为树------------------------------------
    // 分别注册发送目标和机器人控制两个插件，生成行为树结构，XML中定义了各个节点和他们之间的连接关系
    auto tree = factory.createTreeFromFile(bt_xml_path);
    
    // ---------------------------------启动Groot2调试发布器-------------------------
    // 创建 Groot2Publisher 对象，将行为树状态信息发布到指定端口（1667），使Groot2调试工具可以连接并实时控制行为树状态
    const unsigned port = 1667;
    BT::Groot2Publisher publisher(tree,port);

    // ----------------------------行为树主循环------------------------------------------
    // 当ROS2 运行正常时，不断对行为树执行tick操作，实时更新各节点状态
    while(rclcpp::ok())
    {
        tree.tickWhileRunning(std::chrono::milliseconds(10));
    }

    // ----------------------清理退出-------------------------------
    // ROS2退出前调用shutdown释放资源
    rclcpp::shutdown();
    return 0;   

}
