#ifndef BEHAVIOR_TREE_MANAGER_HPP
#define BEHAVIOR_TREE_MANAGER_HPP

#include <behaviortree_cpp/bt_factory.h>
#include <mutex>
#include <shared_mutex> // C++17
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <unordered_map>
#include <string>
#include <memory>
#include <spdlog/spdlog.h>

class BehaviorTreeManager {
public:
    struct TreeInfo {
        std::shared_ptr<BT::Tree> tree;
        std::string name;
        std::atomic<double> last_tick_duration_ms{0.0};
    };

    BehaviorTreeManager() : running_(false), thread_started_(ATOMIC_FLAG_INIT) {}
    ~BehaviorTreeManager() { stop(); }

    // 通过名称添加树，返回是否成功（防止重名）
    bool addTree(const std::string& name, std::shared_ptr<BT::Tree> tree) {
        if (!tree) return false;
        std::unique_lock<std::shared_mutex> lock(tree_mutex_);
        if (tree_map_.find(name) != tree_map_.end()) {
            spdlog::warn("Tree with name '{}' already exists.", name);
            return false;
        }
        
        auto info = std::make_shared<TreeInfo>();
        info->tree = tree;
        info->name = name;
        tree_map_[name] = info;
        dirty_ = true; // 标记需要更新快照
        return true;
    }

    void removeTree(const std::string& name) {
        std::unique_lock<std::shared_mutex> lock(tree_mutex_);
        if (tree_map_.erase(name)) {
            dirty_ = true;
            spdlog::info("Tree '{}' removed.", name);
        }
    }

    void start(unsigned int tick_interval_ms = 100) {
        if (thread_started_.test_and_set()) return;

        running_ = true;
        tick_thread_ = std::thread([this, tick_interval_ms]() {
            // 局部快照，减少锁竞争
            std::vector<std::shared_ptr<TreeInfo>> active_trees;

            while (running_) {
                auto start_time = std::chrono::steady_clock::now();

                // 1. 检查并更新快照 (只有在 dirty_ 时才加写锁)
                updateSnapshotIfNeeded(active_trees);

                // 2. 无锁迭代执行
                for (auto& info : active_trees) {
                    try {
                        auto t1 = std::chrono::steady_clock::now();
                        info->tree->tickExactlyOnce();
                        auto t2 = std::chrono::steady_clock::now();
                        
                        info->last_tick_duration_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
                    } catch (const std::exception& e) {
                        spdlog::error("Tree '{}' crashed: {}", info->name, e.what());
                        removeTree(info->name); // 隔离故障
                    }
                }

                // 3. 智能休眠
                std::unique_lock<std::mutex> wait_lock(wait_mutex_);
                cv_.wait_for(wait_lock, std::chrono::milliseconds(tick_interval_ms), [this] {
                    return !running_.load(); 
                });
            }
        });
    }

    void stop() {
        running_ = false;
        cv_.notify_all();
        if (tick_thread_.joinable()) tick_thread_.join();
        thread_started_.clear();
    }

    // 查询某棵树的运行耗时
    double getTickDuration(const std::string& name) {
        std::shared_lock<std::shared_mutex> lock(tree_mutex_);
        if (tree_map_.count(name)) return tree_map_[name]->last_tick_duration_ms;
        return 0.0;
    }

private:
    void updateSnapshotIfNeeded(std::vector<std::shared_ptr<TreeInfo>>& snapshot) {
        if (dirty_) {
            std::shared_lock<std::shared_mutex> lock(tree_mutex_);
            snapshot.clear();
            for (auto const& [name, info] : tree_map_) {
                snapshot.push_back(info);
            }
            dirty_ = false;
        }
    }

    // 树存储映射：Name -> Info
    std::unordered_map<std::string, std::shared_ptr<TreeInfo>> tree_map_;
    std::shared_mutex tree_mutex_; // 读写锁
    std::atomic<bool> dirty_{false};

    std::thread tick_thread_;
    std::atomic<bool> running_;
    std::atomic_flag thread_started_;
    
    std::condition_variable cv_;
    std::mutex wait_mutex_;
};

#endif