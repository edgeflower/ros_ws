#ifndef BEHAVIOR_TREE_MANAGER_HPP
#define BEHAVIOR_TREE_MANAGER_HPP

#include <behaviortree_cpp/bt_factory.h>
#include <mutex>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <algorithm>

// 行为树管理器：集中管理所有行为树实例，并周期性tick
class BehaviorTreeManager
{
public:
    BehaviorTreeManager() : running_(false) {}
    ~BehaviorTreeManager() { stop(); }

    // 添加一棵行为树实例
    void addTree(BT::Tree* tree)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        trees_.push_back(tree);
    }

    // 移除一棵行为树实例
    void removeTree(BT::Tree* tree)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = std::remove(trees_.begin(), trees_.end(), tree);
        trees_.erase(it, trees_.end());
    }

    // 启动管理器，tick周期单位为毫秒（默认100ms）
    void start(unsigned int tick_interval_ms = 100)
    {
        running_ = true;
        tick_thread_ = std::thread([this, tick_interval_ms]()
        {
            while(running_)
            {
                tickAll();
                std::this_thread::sleep_for(std::chrono::milliseconds(tick_interval_ms));
            }
        });
    }

    // 停止tick线程
    void stop()
    {
        running_ = false;
        if(tick_thread_.joinable())
            tick_thread_.join();
    }

    // 对所有行为树进行一次tick操作
    void tickAll()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for(auto& tree : trees_)
        {
            tree->tickWhileRunning(std::chrono::milliseconds(10));
        }
    }

private:
    std::vector<BT::Tree*> trees_; // 存储所有行为树指针
    std::mutex mutex_;             // 互斥量保护trees_
    std::thread tick_thread_;      // 定时tick线程
    std::atomic<bool> running_;    // 运行标志
};

#endif // BEHAVIOR_TREE_MANAGER_HPP
