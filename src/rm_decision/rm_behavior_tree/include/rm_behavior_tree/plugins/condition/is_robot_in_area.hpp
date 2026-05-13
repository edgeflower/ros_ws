#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_IN_AREA_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_IN_AREA_HPP_
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <rm_decision_interfaces/msg/robot_area_status.hpp>
namespace rm_behavior_tree{
/**
* @brief Action 节点，用于判断机器人是否在指定区域内
* 该节点从输入端口获取机器人当前位置和目标区域信息，并根据条件判断机器人是否在目标区域内。
* 如果机器人在区域内，返回成功；否则返回失败。
* @param[in] robot_area_status 机器人区域状态消息
* @param[in] target_area_name 目标区域名称
* @param[in] target_area_type 目标区域类型（可选，如果需要根据区域类型进行更细粒度的判断）
* @param[in] heat_threshold 最大热量阈值 （哨兵最大热量 400）
*/
class IsRobotInArea : public BT::SimpleConditionNode{
    public:
    IsRobotInArea(const std::string & name, const BT::NodeConfig & config);

    // BT::NodeStatus checkGameStart(BT::TreeNode & self_node)
    BT::NodeStatus checkRobotInArea();
    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<rm_decision_interfaces::msg::RobotAreaStatus>("robot_area_status"),
            BT::InputPort<std::string>("target_area_name"),
            BT::InputPort<std::string>("target_area_type") // 可选输入，如果需要根据区域类型进行更细粒度的判断
            //BT::InputPort<int>("heat_threshold")
        };
    }

    private:
    std::string target_area_name;
    std::string target_area_type;
};


} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_IN_AREA_HPP_