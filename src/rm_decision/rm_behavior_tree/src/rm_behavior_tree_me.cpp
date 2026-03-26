#include "rm_behavior_tree/rm_behavior_tree.hpp"
#include "rm_behavior_tree/BehaviorTreeManager.hpp"
#include <memory>

int main(int argc, char ** argv)
{
    // 初始化ROS2系统
    rclcpp::init(argc, argv);

    // 创建自定义行为树节点（继承自rclcpp::Node）
    auto bt_node = std::make_shared<rm_behavior_tree::BehaviorTreeNode>();

    // 创建行为树管理器
    BehaviorTreeManager bt_manager;
    // 通过 bt_node 的 getter 获取内部加载好的行为树，并添加到管理器中
    bt_manager.addTree(&bt_node->getTree());

    // 启动行为树管理器，设置每100毫秒tick一次所有行为树
    bt_manager.start(1000);

    // 创建多线程执行器，处理ROS2的各类回调（定时器、订阅、服务等）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(bt_node);

    // 进入ROS2事件循环
    executor.spin();

    // 当ROS2退出时，停止行为树管理器的tick线程并清理资源
    bt_manager.stop();
    rclcpp::shutdown();
    return 0;
}


