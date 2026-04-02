#include "rm_behavior_tree/plugins/action/bag_recorder.hpp"
#include <rclcpp/logging.hpp>
#include <sstream>
#include <filesystem>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#include <fcntl.h>

namespace rm_behavior_tree
{

BagRecorderAction::BagRecorderAction(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf),
  recording_pid_(-1),
  is_recording_(false)
{
}

BagRecorderAction::~BagRecorderAction()
{
  stopRecording();
}

BT::NodeStatus BagRecorderAction::tick()
{
  uint8_t end_game_progress = 5;
  getInput("end_game_progress", end_game_progress);

  // 从黑板获取游戏状态
  rm_decision_interfaces::msg::GameStatus game_status;
  if (!getInput("game_status", game_status)) {
    // 还没有收到游戏状态消息
    return BT::NodeStatus::RUNNING;
  }

  uint8_t current_progress = game_status.game_progress;

  if (current_progress == end_game_progress) {
    // 比赛结束，停止录制
    if (is_recording_) {
      RCLCPP_INFO(rclcpp::get_logger("BagRecorder"), "比赛结束 (game_progress=%d)，停止录制", current_progress);
      stopRecording();
      is_recording_ = false;
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 比赛进行中，检查是否需要录制
  if (!is_recording_) {
    std::string topics_str, output_dir = "/tmp/rosbags", bag_prefix = "record", qos_config_path = "";

    if (!getInput("topics", topics_str)) {
      RCLCPP_ERROR(rclcpp::get_logger("BagRecorder"), "未指定topics参数");
      return BT::NodeStatus::FAILURE;
    }

    getInput("output_dir", output_dir);
    getInput("bag_prefix", bag_prefix);
    getInput("qos_config_path", qos_config_path);

    auto topics = parseTopics(topics_str);
    if (topics.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("BagRecorder"), "话题列表为空");
      return BT::NodeStatus::FAILURE;
    }

    // 记录日志
    std::stringstream ss;
    ss << "比赛进行中 (game_progress=" << (int)current_progress << ")，开始录制话题: ";
    for (const auto & topic : topics) {
      ss << topic << " ";
    }
    RCLCPP_INFO(rclcpp::get_logger("BagRecorder"), "%s", ss.str().c_str());

    if (startRecording(topics, output_dir, bag_prefix, qos_config_path)) {
      is_recording_ = true;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }

  // 检查录制进程是否还在运行
  if (is_recording_ && recording_pid_ > 0) {
    if (kill(recording_pid_, 0) != 0) {
      // 进程已意外结束
      RCLCPP_WARN(rclcpp::get_logger("BagRecorder"), "录制进程意外结束");
      is_recording_ = false;
      recording_pid_ = -1;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

std::vector<std::string> BagRecorderAction::parseTopics(const std::string & topics_str)
{
  std::vector<std::string> topics;
  std::stringstream ss(topics_str);
  std::string topic;

  while (std::getline(ss, topic, ',')) {
    // 去除空格
    topic.erase(0, topic.find_first_not_of(" \t\r\n"));
    topic.erase(topic.find_last_not_of(" \t\r\n") + 1);
    if (!topic.empty()) {
      topics.push_back(topic);
    }
  }

  return topics;
}

bool BagRecorderAction::startRecording(
  const std::vector<std::string> & topics,
  const std::string & output_dir,
  const std::string & prefix,
  const std::string & qos_path)
{
  // 1. 创建输出路径
  std::filesystem::create_directories(output_dir);
  std::string full_path = output_dir + "/" + prefix;

  // 2. Fork 子进程
  recording_pid_ = fork();

  if (recording_pid_ == 0) {
    // === 子进程逻辑 ===

    // 重定向标准输出和错误到 /dev/null，防止污染终端
    int dev_null = open("/dev/null", O_WRONLY);
    if (dev_null >= 0) {
      dup2(dev_null, STDOUT_FILENO);
      dup2(dev_null, STDERR_FILENO);
      close(dev_null);
    }

    // 创建新会话，使子进程成为会话领导者
    setsid();

    // 构建参数列表
    std::vector<char*> args;

    // 基础命令
    args.push_back(const_cast<char*>("ros2"));
    args.push_back(const_cast<char*>("bag"));
    args.push_back(const_cast<char*>("record"));

    // QoS配置
    std::string qos_arg;
    if (!qos_path.empty()) {
      qos_arg = "--qos-profile-overrides-path=" + qos_path;
      args.push_back(const_cast<char*>(qos_arg.c_str()));
    }

    // 输出路径
    std::string output_arg = "-o";
    args.push_back(const_cast<char*>(output_arg.c_str()));
    args.push_back(const_cast<char*>(full_path.c_str()));

    // 添加话题
    for (const auto & topic : topics) {
      args.push_back(const_cast<char*>(topic.c_str()));
    }

    args.push_back(nullptr);  // 参数数组必须以 nullptr 结尾

    // 执行命令
    execvp("ros2", args.data());

    // 如果 execvp 返回，说明执行失败
    _exit(1);

  } else if (recording_pid_ > 0) {
    // === 父进程逻辑 ===
    RCLCPP_INFO(rclcpp::get_logger("BagRecorder"), "录制已启动，PID: %d", recording_pid_);
    return true;
  } else {
    // Fork 失败
    RCLCPP_ERROR(rclcpp::get_logger("BagRecorder"), "fork() 失败");
    recording_pid_ = -1;
    return false;
  }
}

bool BagRecorderAction::stopRecording()
{
  if (recording_pid_ <= 0) {
    return true;  // 没有录制进程
  }

  // 检查进程是否还活着
  if (kill(recording_pid_, 0) == 0) {
    RCLCPP_INFO(rclcpp::get_logger("BagRecorder"), "正在停止录制 (PID: %d)...", recording_pid_);

    // 发送 SIGINT (模拟 Ctrl+C)，这对 rosbag 写入索引非常重要
    kill(recording_pid_, SIGINT);

    // 非阻塞等待，最多等待2秒
    int status;
    for (int i = 0; i < 20; ++i) {
      pid_t result = waitpid(recording_pid_, &status, WNOHANG);
      if (result == recording_pid_) {
        // 进程已正常退出
        recording_pid_ = -1;
        RCLCPP_INFO(rclcpp::get_logger("BagRecorder"), "录制已正常停止");
        return true;
      }
      if (result == -1) {
        // 出错或进程已不存在
        break;
      }
      usleep(100000);  // 100ms
    }

    // 如果还没死，强行杀死
    RCLCPP_WARN(rclcpp::get_logger("BagRecorder"), "录制进程未响应，强制终止");
    kill(recording_pid_, SIGKILL);
    waitpid(recording_pid_, &status, 0);
  }

  recording_pid_ = -1;
  return true;
}

}  // namespace rm_behavior_tree

// BT 插件注册
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<rm_behavior_tree::BagRecorderAction>("BagRecorder");
}
