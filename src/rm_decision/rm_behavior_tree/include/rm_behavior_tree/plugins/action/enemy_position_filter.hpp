#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__ENEMY_POSITION_FILTER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__ENEMY_POSITION_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <behaviortree_cpp/action_node.h>
#include <deque>
#include <cmath>
#include <armor_interfaces/msg/target.hpp>

namespace rm_behavior_tree
{
    /**
     * @brief 对敌人位置进行滤波的节点 (数据清洗层)
     *
     * 职责边界：
     *   - 只负责平滑坐标和剔除跳变
     *   - 不关心偏移距离和机器人位置 (由 ArmorToGoal 处理)
     *   - 透传 confidence, tracking, id 等元数据
     *
     * 与 ArmorToGoal 的配合:
     *   - EnemyPositionFilter 输出完整的 Target 消息
     *   - ArmorToGoal 接收 "被洗过坐标" 的 Target 进行战术计算
     *
     * @param[in] enemy_position 输入的原始目标 (armor_interfaces::msg::Target)
     * @param[out] filtered_position 滤波后的完整目标 (armor_interfaces::msg::Target)
     * @param[in] alpha 滤波系数 (0.0-1.0)，默认0.3。调大=更快响应，调小=更平滑
     * @param[in] max_distance 异常值阈值 (米)，默认1.5 (RoboMaster 3v3)
     * @param[in] history_size 历史数据窗口大小，用于异常检测
     * @param[in] zero_threshold 零值判定阈值 (米)，小于此值视为无效检测
     */

class EnemyPositionFilter : public BT::SyncActionNode
{
    public:
        EnemyPositionFilter(const std::string& name, const BT::NodeConfig& config);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<armor_interfaces::msg::Target>("enemy_position"),
                // CHANGED: Output full Target message instead of PointStamped
                // This preserves confidence, tracking, id, armors_num, etc.
                BT::OutputPort<armor_interfaces::msg::Target>("filtered_position"),
                BT::InputPort<double>("alpha", 0.3, "滤波系数 (0.0-1.0)，默认0.3"),
                // CHANGED: Default from 10.0m to 1.5m for RoboMaster 3v3
                // Single frame jump of 10m is definitely misidentification
                BT::InputPort<double>("max_distance", 1.5, "异常值阈值(米)，默认1.5 (RoboMaster 3v3)"),
                BT::InputPort<int>("history_size", 5, "历史数据窗口大小"),
                BT::InputPort<double>("zero_threshold", 0.1, "0值判断阈值(米)，小于此值视为无效检测"),
                BT::InputPort<double>("min_output_interval", 0.1, "最小输出间隔(秒)，默认0.1 (10Hz)"),
                BT::InputPort<double>("confidence_half_life", 1.0, "置信度半衰期(秒)，默认1.0。经过此时间confidence衰减为一半"),
                BT::InputPort<double>("min_confidence", 0.0, "最小置信度阈值，默认0.0。衰减到该值后不再继续衰减")
            };
        }

        BT::NodeStatus tick() override;
        void reset();

    private:
        // CHANGED: Store full Target message instead of just PointStamped
        armor_interfaces::msg::Target filtered_target_;

        // 滤波后的位置 (用于计算，filtered_target_ 包含完整信息)
        struct FilteredPosition {
            double x, y;
            rclcpp::Time last_update;
        } filtered_position_;

        // 指数移动平均滤波器
        double alpha_ = 0.3;  // 滤波系数，0.3 = 平滑响应与实时性的平衡
        bool initialized_ = false;

        // 输出频率限制 (方案1)
        rclcpp::Time last_output_time_{0, 0, RCL_ROS_TIME};
        double min_output_interval_ = 0.1;  // 默认100ms (10Hz)

        // 历史数据用于异常检测
        struct PositionHistory {
            rclcpp::Time time;
            double x, y;
        };
        std::deque<PositionHistory> history_;
        size_t max_history_size_ = 5;

        // Confidence 指数衰减
        double original_confidence_ = 0.0;  // 最后收到的原始置信度
        rclcpp::Time last_valid_confidence_time_{0, 0, RCL_ROS_TIME};  // 最后收到有效置信度的时间
        double confidence_half_life_ = 1.0;  // 置信度半衰期(秒)
        double min_confidence_ = 0.0;  // 最小置信度阈值

        // 辅助函数
        double calculateDistance(const PositionHistory& p1, const PositionHistory& p2) const;
        bool isOutlier(const armor_interfaces::msg::Target& pos) const;
        void updateFilteredPosition(const armor_interfaces::msg::Target& new_pos);

        // Helper: Create filtered target with new position but preserved metadata
        armor_interfaces::msg::Target createFilteredTarget(
            double x, double y,
            const armor_interfaces::msg::Target& original);

        // Helper: Create default target when no valid data available
        // Sets confidence=0.0 to signal no enemy detected to downstream nodes
        armor_interfaces::msg::Target createDefaultTarget();

        // Helper: Calculate decayed confidence based on elapsed time
        // Uses exponential decay: confidence = original * exp(-lambda * elapsed)
        double calculateDecayedConfidence() const;
    };

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__ENEMY_POSITION_FILTER_HPP_
