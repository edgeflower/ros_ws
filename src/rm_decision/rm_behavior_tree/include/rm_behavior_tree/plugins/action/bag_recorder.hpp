#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__BAG_RECORDER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__BAG_RECORDER_HPP_

#include <behaviortree_cpp/action_node.h>
#include <string>
#include <vector>
#include <memory>
#include <sys/types.h>
#include <rclcpp/rclcpp.hpp>
#include <rm_decision_interfaces/msg/game_status.hpp>

namespace rm_behavior_tree
{

class BagRecorderAction : public BT::SyncActionNode
{
public:
  BagRecorderAction(const std::string & name, const BT::NodeConfig & conf);
  ~BagRecorderAction();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<rm_decision_interfaces::msg::GameStatus>("game_status_current", "游戏状态（从黑板读取）"),
      BT::InputPort<std::string>("topics", "逗号分隔的话题列表，如: /livox/lidar,/livox/imu"),
      BT::InputPort<std::string>("output_dir", "/tmp/rosbags", "bag输出目录"),
      BT::InputPort<std::string>("bag_prefix", "record", "bag文件前缀"),
      BT::InputPort<std::string>("qos_config_path", "", "QoS配置文件路径(可选)"),
      BT::InputPort<uint8_t>("end_game_progress", "5", "比赛结束时的game_progress值")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::vector<std::string> parseTopics(const std::string & topics_str);
  bool startRecording(const std::vector<std::string> & topics,
                      const std::string & output_dir,
                      const std::string & prefix,
                      const std::string & qos_path);
  bool stopRecording();

  pid_t recording_pid_;
  bool is_recording_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__BAG_RECORDER_HPP_
