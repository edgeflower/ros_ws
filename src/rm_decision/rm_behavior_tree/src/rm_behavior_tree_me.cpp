#include "rm_behavior_tree/rm_behavior_tree.hpp"
#include "rm_behavior_tree/BehaviorTreeManager.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // 1. 创建 ROS2 节点 (使用 shared_ptr)
    auto bt_node = std::make_shared<rm_behavior_tree::BehaviorTreeNode>();

    // 2. 初始化节点（注册插件并创建行为树）
    // 必须在 shared_ptr 创建后调用，因为内部使用 shared_from_this()
    bt_node->init();

    // 3. 创建管理器
    BehaviorTreeManager bt_manager;

    // 给这棵树起个名字，方便监控和日志记录
    auto tree_ptr = bt_node->getTreePtr();
    if (tree_ptr) {
        bt_manager.addTree("MainTaskTree", tree_ptr);
    } else {
        RCLCPP_ERROR(bt_node->get_logger(), "Failed to get Behavior Tree pointer!");
        return -1;
    }

    // 4. 启动管理器
    // 注意：100ms (0.1秒) 对机器人控制来说可能太慢了，通常设为 50-100ms
    bt_manager.start(100);

    // 5. 设置多线程执行器
    // ROS2 的回调（如传感器数据订阅）在这些线程中运行
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(bt_node);

    RCLCPP_INFO(bt_node->get_logger(), "System started. BT Manager and ROS2 Executor are running.");

    // 6. 进入循环
    // 这会阻塞主线程，处理所有 ROS2 相关逻辑
    executor.spin();

    // 7. 清理退出
    RCLCPP_INFO(bt_node->get_logger(), "Shutting down...");
    bt_manager.stop();
    rclcpp::shutdown();

    return 0;
}
