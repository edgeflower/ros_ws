/**
 * @file utility_ai_node.cpp
 * @brief 哨兵 Utility AI 战术决策节点
 *
 * 这个节点是哨兵机器人的"大脑"——它根据战场信息，用 Utility AI 的方式
 * 给每个可能的战术行为打分，然后选择分数最高的那个执行。
 *
 * == Utility AI 是什么？ ==
 * 简单说就是：每个行为（攻击、撤退、补给、巡逻、防守）都有一个评分函数，
 * 根据当前状态计算分数。分数最高的行为被选中执行。
 * 这比行为树或状态机更灵活，因为每个行为独立评分，容易调试和调参。
 *
 * == 输入（订阅的话题） ==
 * 1. /game_status       - 比赛状态（是否在比赛中）
 * 2. /robot_status      - 机器人状态（血量、热量、是否被攻击）
 * 3. /target_tracking   - 敌人目标追踪（装甲板检测信息）
 * 4. /lidar_odometry    - 里程计（机器人当前位置和速度）
 * 5. /robot_area_status - 区域检测结果（机器人在哪个语义区域）
 * 6. /enemy_in_forbidden_area - 敌人是否在禁区（可选）
 *
 * == 输出（发布的话题） ==
 * /sentry_decision - 决策结果（模式、评分、原因、目标点）
 *
 * == 工作流程 ==
 * 1. 各 callback 只更新缓存数据（不在这里做决策）
 * 2. 定时器以 10Hz 频率触发 computeDecision()
 * 3. computeDecision() 计算各行为评分，选择最优行为，发布结果
 */

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// rm_decision_interfaces 消息
#include "rm_decision_interfaces/msg/game_status.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include "rm_decision_interfaces/msg/robot_area_status.hpp"
#include "rm_decision_interfaces/msg/enemy_forbidden_area.hpp"
#include <rm_decision_interfaces/msg/sentry_decision.hpp>

// armor_interfaces 敌人追踪消息（与 /target_tracking 话题类型一致）
#include "armor_interfaces/msg/target.hpp"

using namespace std::chrono_literals;

// ============================================================================
// 常量定义：战术模式名称
// ============================================================================
static const std::string MODE_IDLE        = "IDLE";
static const std::string MODE_ATTACK      = "ATTACK";
static const std::string MODE_RETREAT     = "RETREAT";
static const std::string MODE_SUPPLY      = "SUPPLY";
static const std::string MODE_PATROL      = "PATROL";
static const std::string MODE_DEFEND_BASE = "DEFEND_BASE";

// ============================================================================
// 缓存数据结构体
// 每个结构体保存消息数据 + 收到该消息的时间戳
// 这样可以判断数据是否过期（超时）
// ============================================================================

/**
 * @brief 比赛状态缓存
 * game_progress: 比赛阶段，4 表示正式比赛中（允许战术决策）
 * stage_remain_time: 当前阶段剩余时间（秒）
 */
struct GameStatusCache {
  rm_decision_interfaces::msg::GameStatus msg;
  rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

/**
 * @brief 机器人状态缓存
 * current_hp: 当前血量
 * shooter_heat: 当前枪口热量（热量越高说明近期射击越多，接近上限需要降温）
 * is_attacked: 是否正在被攻击（用于判断是否需要撤退）
 * shot_allowance: 17mm 弹丸剩余量
 */
struct RobotStatusCache {
  rm_decision_interfaces::msg::RobotStatus msg;
  rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

/**
 * @brief 敌人目标追踪缓存
 * tracking: 是否正在追踪目标（true = 看到敌人）
 * armors_num: 识别到的装甲板数量（0 = 无目标）
 * position: 敌人位置（3D坐标）
 * confidence: 检测置信度（0.0 ~ 1.0）
 */
struct TargetCache {
  armor_interfaces::msg::Target msg;
  rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

/**
 * @brief 里程计缓存（机器人自身位置和速度）
 * 位置在 odom 坐标系下，需要通过 tf2 转换到 map 坐标系
 */
struct OdomCache {
  nav_msgs::msg::Odometry msg;
  rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

/**
 * @brief 区域检测结果缓存
 * is_in_area: 机器人是否在任何已定义的区域内
 * matched_area_types: 所有匹配到的区域类型列表
 *   可能的类型: forbidden, protect, patrol, attack
 *   以及: danger, high_ground, rough_road, slope, retreat, supply
 */
struct AreaStatusCache {
  rm_decision_interfaces::msg::RobotAreaStatus msg;
  rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

/**
 * @brief 敌人禁区检测结果缓存
 * is_forbidden: true 表示敌人在禁区内
 * 注意：这个订阅是可选的（通过参数控制）
 */
struct EnemyForbiddenCache {
  rm_decision_interfaces::msg::EnemyForbiddenArea msg;
  rclcpp::Time recv_time{0, 0, RCL_ROS_TIME};
  bool valid = false;
};

// ============================================================================
// 主节点类
// ============================================================================
class UtilityAiNode : public rclcpp::Node {
public:
  explicit UtilityAiNode(const rclcpp::NodeOptions & options)
  : Node("utility_ai_node", options)
  {
    // ------------------------------------------------------------------
    // 声明并读取所有参数
    // 参数可以在 launch 文件或 yaml 中覆盖，方便调参
    // ------------------------------------------------------------------
    declare_parameter("decision_rate_hz", 10.0);         // 决策频率（Hz）
    declare_parameter("switch_margin", 10.0);             // 滞回防抖阈值（防止模式频繁切换）
    declare_parameter("enemy_visible_timeout", 1.0);      // 敌人可见超时（秒）
    declare_parameter("low_hp_ratio", 0.35);              // 低血量比例（低于此值视为危险）
    declare_parameter("critical_hp_ratio", 0.20);         // 极低血量比例（低于此值必须撤退）
    declare_parameter("attack_distance_min", 0.8);        // 最佳攻击距离下限（米）
    declare_parameter("attack_distance_max", 5.0);        // 最佳攻击距离上限（米）
    declare_parameter("danger_damage_window", 3.0);       // 伤害统计时间窗口（秒）
    declare_parameter("damage_retreat_threshold", 120.0); // 伤害撤退阈值（窗口内掉血超过此值就撤退）
    declare_parameter("maximum_hp", 600.0);               // 机器人最大血量（RobotStatus 不含此字段，需配置）
    declare_parameter("shooter_heat_limit", 240.0);       // 枪口热量上限（用于计算热量比例）
    declare_parameter("enable_enemy_forbidden_sub", false);// 是否订阅敌人禁区话题
    declare_parameter("robot_area", "bumpy_area");          // 机器人所在的起伏路段类型（用于调参）

    // 各战术目标点坐标（map 坐标系下，单位：米）
    // 默认值为 0.0，必须通过 yaml 配置实际坐标
    declare_parameter("retreat_goal_x", 0.0);
    declare_parameter("retreat_goal_y", 0.0);
    declare_parameter("supply_goal_x", 0.0);
    declare_parameter("supply_goal_y", 0.0);
    declare_parameter("defend_goal_x", 0.0);
    declare_parameter("defend_goal_y", 0.0);
    declare_parameter("patrol_goal_x", 0.0);
    declare_parameter("patrol_goal_y", 0.0);

    // 读取参数到成员变量
    decision_rate_hz_         = get_parameter("decision_rate_hz").as_double();
    switch_margin_            = get_parameter("switch_margin").as_double();
    enemy_visible_timeout_    = get_parameter("enemy_visible_timeout").as_double();
    low_hp_ratio_             = get_parameter("low_hp_ratio").as_double();
    critical_hp_ratio_        = get_parameter("critical_hp_ratio").as_double();
    attack_distance_min_      = get_parameter("attack_distance_min").as_double();
    attack_distance_max_      = get_parameter("attack_distance_max").as_double();
    danger_damage_window_     = get_parameter("danger_damage_window").as_double();
    damage_retreat_threshold_ = get_parameter("damage_retreat_threshold").as_double();
    maximum_hp_               = get_parameter("maximum_hp").as_double();
    shooter_heat_limit_       = get_parameter("shooter_heat_limit").as_double();
    enable_enemy_forbidden_   = get_parameter("enable_enemy_forbidden_sub").as_bool();
    robot_area_              = get_parameter("robot_area").as_string();
    retreat_goal_x_  = get_parameter("retreat_goal_x").as_double();
    retreat_goal_y_  = get_parameter("retreat_goal_y").as_double();
    supply_goal_x_   = get_parameter("supply_goal_x").as_double();
    supply_goal_y_   = get_parameter("supply_goal_y").as_double();
    defend_goal_x_   = get_parameter("defend_goal_x").as_double();
    defend_goal_y_   = get_parameter("defend_goal_y").as_double();
    patrol_goal_x_   = get_parameter("patrol_goal_x").as_double();
    patrol_goal_y_   = get_parameter("patrol_goal_y").as_double();

    RCLCPP_INFO(get_logger(), "Utility AI Node 初始化完成");
    RCLCPP_INFO(get_logger(), "  决策频率: %.1f Hz, 滞回阈值: %.1f", decision_rate_hz_, switch_margin_);
    RCLCPP_INFO(get_logger(), "  低血量比例: %.2f, 极低血量比例: %.2f", low_hp_ratio_, critical_hp_ratio_);
    RCLCPP_INFO(get_logger(), "  最大HP: %.0f, 热量上限: %.0f", maximum_hp_, shooter_heat_limit_);
    RCLCPP_INFO(get_logger(), "  敌人禁区订阅: %s", enable_enemy_forbidden_ ? "开启" : "关闭");

    // ------------------------------------------------------------------
    // 初始化 TF2（用于坐标变换，将 odom 坐标转到 map 坐标）
    // ------------------------------------------------------------------
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ------------------------------------------------------------------
    // 创建订阅者
    // 每个订阅者收到消息后，只更新对应的缓存数据，不做复杂处理
    // ------------------------------------------------------------------
    // 比赛状态订阅
    game_status_sub_ = create_subscription<rm_decision_interfaces::msg::GameStatus>(
      "/game_status", rclcpp::QoS(10),
      [this](const rm_decision_interfaces::msg::GameStatus::SharedPtr msg) {
        game_status_.msg = *msg;
        game_status_.recv_time = now();
        game_status_.valid = true;
      });

    // 机器人状态订阅
    robot_status_sub_ = create_subscription<rm_decision_interfaces::msg::RobotStatus>(
      "/robot_status", rclcpp::QoS(10),
      [this](const rm_decision_interfaces::msg::RobotStatus::SharedPtr msg) {
        // 记录血量变化，用于统计掉血速度
        if (robot_status_.valid) {
          uint16_t old_hp = robot_status_.msg.current_hp;
          uint16_t new_hp = msg->current_hp;
          if (new_hp < old_hp) {
            // 血量减少了，记录掉血量和时间
            damage_history_.push_back({now(), static_cast<double>(old_hp - new_hp)});
          }
        }
        robot_status_.msg = *msg;
        robot_status_.recv_time = now();
        robot_status_.valid = true;
      });

    // 敌人目标追踪订阅（使用 armor_interfaces::msg::Target，与现有系统一致）
    target_sub_ = create_subscription<armor_interfaces::msg::Target>(
      "/target_tracking", rclcpp::QoS(10),
      [this](const armor_interfaces::msg::Target::SharedPtr msg) {
        target_.msg = *msg;
        target_.recv_time = now();
        target_.valid = true;
      });

    // 里程计订阅
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lidar_odometry", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_.msg = *msg;
        odom_.recv_time = now();
        odom_.valid = true;
      });

    // 区域检测订阅
    area_status_sub_ = create_subscription<rm_decision_interfaces::msg::RobotAreaStatus>(
      "/robot_area_status", rclcpp::QoS(10),
      [this](const rm_decision_interfaces::msg::RobotAreaStatus::SharedPtr msg) {
        area_status_.msg = *msg;
        area_status_.recv_time = now();
        area_status_.valid = true;
      });

    // 敌人禁区检测订阅（可选）
    if (enable_enemy_forbidden_) {
      enemy_forbidden_sub_ = create_subscription<rm_decision_interfaces::msg::EnemyForbiddenArea>(
        "/enemy_in_forbidden_area", rclcpp::QoS(10),
        [this](const rm_decision_interfaces::msg::EnemyForbiddenArea::SharedPtr msg) {
          enemy_forbidden_.msg = *msg;
          enemy_forbidden_.recv_time = now();
          enemy_forbidden_.valid = true;
        });
      RCLCPP_INFO(get_logger(), "  已启用敌人禁区订阅");
    }

    // ------------------------------------------------------------------
    // 创建发布者
    // ------------------------------------------------------------------
    decision_pub_ = create_publisher<rm_decision_interfaces::msg::SentryDecision>(
      "/sentry_decision", rclcpp::QoS(10));

    // ------------------------------------------------------------------
    // 创建定时器
    // 以固定频率（默认 10Hz）执行决策计算
    // ------------------------------------------------------------------
    auto timer_period = std::chrono::duration<double>(1.0 / decision_rate_hz_);
    decision_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&UtilityAiNode::computeDecision, this));

    RCLCPP_INFO(get_logger(), "  定时器已启动，周期: %.1f ms", timer_period.count() * 1000.0);
  }

private:
  // ====================================================================
  // 成员变量：参数
  // ====================================================================
  double decision_rate_hz_ = 10.0;          // 决策频率
  double switch_margin_ = 10.0;             // 滞回防抖阈值
  double enemy_visible_timeout_ = 1.0;      // 敌人可见超时
  double low_hp_ratio_ = 0.35;              // 低血量比例
  double critical_hp_ratio_ = 0.20;         // 极低血量比例
  double attack_distance_min_ = 0.8;        // 攻击距离下限
  double attack_distance_max_ = 5.0;        // 攻击距离上限
  double danger_damage_window_ = 3.0;       // 伤害统计窗口
  double damage_retreat_threshold_ = 120.0; // 伤害撤退阈值
  double maximum_hp_ = 600.0;               // 最大血量
  double shooter_heat_limit_ = 240.0;       // 热量上限
  bool   enable_enemy_forbidden_ = false;   // 是否启用敌人禁区订阅
  std::string robot_area_ = "bumpy_area";   // 起伏路段

  // 各战术目标点
  double retreat_goal_x_ = 0.0, retreat_goal_y_ = 0.0;
  double supply_goal_x_  = 0.0, supply_goal_y_  = 0.0;
  double defend_goal_x_  = 0.0, defend_goal_y_  = 0.0;
  double patrol_goal_x_  = 0.0, patrol_goal_y_  = 0.0;

  // ====================================================================
  // 成员变量：缓存数据
  // ====================================================================
  GameStatusCache   game_status_;
  RobotStatusCache  robot_status_;
  TargetCache       target_;
  OdomCache         odom_;
  AreaStatusCache   area_status_;
  EnemyForbiddenCache enemy_forbidden_;

  /**
   * @brief 掉血历史记录
   * 每次检测到血量下降时，记录 {时间, 掉血量}
   * 用于统计在 danger_damage_window_ 时间窗口内的总掉血量
   */
  std::vector<std::pair<rclcpp::Time, double>> damage_history_;

  // ====================================================================
  // 成员变量：当前决策状态
  // ====================================================================
  std::string current_mode_ = MODE_IDLE;     // 当前执行的模式
  double      current_score_ = 0.0;          // 当前模式的评分

  /**
   * @brief 上一次有效的机器人位置（map 坐标系）
   * 当 tf2 转换失败时使用此位置作为回退
   */
  double last_robot_x_ = 0.0;
  double last_robot_y_ = 0.0;
  bool   has_last_position_ = false;

  // ====================================================================
  // ROS 接口
  // ====================================================================
  rclcpp::Subscription<rm_decision_interfaces::msg::GameStatus>::SharedPtr      game_status_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::RobotStatus>::SharedPtr     robot_status_sub_;
  rclcpp::Subscription<armor_interfaces::msg::Target>::SharedPtr                target_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                      odom_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::RobotAreaStatus>::SharedPtr  area_status_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::EnemyForbiddenArea>::SharedPtr enemy_forbidden_sub_;

  rclcpp::Publisher<rm_decision_interfaces::msg::SentryDecision>::SharedPtr decision_pub_;
  rclcpp::TimerBase::SharedPtr decision_timer_;

  // TF2 坐标变换
  std::shared_ptr<tf2_ros::Buffer>                tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>     tf_listener_;

  // ====================================================================
  // 工具函数
  // ====================================================================

  /**
   * @brief 判断数据是否在有效期内（未超时）
   * @param timestamp 数据收到时的时间戳
   * @param timeout_sec 超时阈值（秒）
   * @return true 表示数据仍然有效
   */
  bool isDataFresh(const rclcpp::Time & timestamp, double timeout_sec) const
  {
    if (timestamp.nanoseconds() == 0) {return false;}
    return (now() - timestamp).seconds() < timeout_sec;
  }

  /**
   * @brief 获取血量比例 (0.0 ~ 1.0)
   * current_hp 来自 RobotStatus，maximum_hp 来自参数配置
   */
  double getHpRatio() const
  {
    if (!robot_status_.valid || maximum_hp_ <= 0.0) {return 1.0;}
    return std::clamp(
      static_cast<double>(robot_status_.msg.current_hp) / maximum_hp_,
      0.0, 1.0);
  }

  /**
   * @brief 获取热量比例 (0.0 ~ 1.0)
   * shooter_heat 来自 RobotStatus，shooter_heat_limit 来自参数配置
   * 比例越高说明枪口越热，接近上限时应该考虑补给/降温
   */
  double getHeatRatio() const
  {
    if (!robot_status_.valid || shooter_heat_limit_ <= 0.0) {return 0.0;}
    return std::clamp(
      static_cast<double>(robot_status_.msg.shooter_heat) / shooter_heat_limit_,
      0.0, 1.0);
  }

  /**
   * @brief 判断是否能看到敌人
   * 条件：最近 enemy_visible_timeout_ 秒内收到有效的目标追踪消息，
   *       且 tracking=true 或 armors_num > 0
   */
  bool isEnemyVisible() const
  {
    if (!target_.valid) {return false;}
    if (!isDataFresh(target_.recv_time, enemy_visible_timeout_)) {return false;}
    return target_.msg.tracking || target_.msg.armors_num > 0;
  }

  /**
   * @brief 从 Target 消息中提取敌人位置（在 map 坐标系下）
   *
   * 适配函数：处理 armor_interfaces::msg::Target 的字段
   * - armors_num > 0 表示有目标
   * - position 是目标的 3D 位置
   * - 该位置在 target 消息的 header.frame_id 坐标系下（通常是 gimbal_yaw）
   * - 必须通过 TF2 变换到 map 坐标系
   *
   * 坐标变换公式：map_pos = R(yaw) * src_pos + translation
   *
   * @param[out] ex, ey 敌人在 map 坐标系下的 x, y 坐标
   * @return true 表示成功提取到敌人位置（含 TF2 变换成功）
   */
  bool extractEnemyTarget(double & ex, double & ey) const
  {
    if (!isEnemyVisible()) {return false;}

    // armors_num == 0 表示没有检测到装甲板
    if (target_.msg.armors_num == 0) {return false;}

    // 从 target 消息的 header.frame_id 获取源坐标系（通常是 gimbal_yaw）
    const std::string & source_frame = target_.msg.header.frame_id;
    // 如果 frame_id 为空，回退到 "gimbal_yaw"
    const std::string src_frame = source_frame.empty() ? "gimbal_yaw" : source_frame;

    const double src_x = target_.msg.position.x;
    const double src_y = target_.msg.position.y;

    // 如果源坐标系已经是 map，直接返回
    if (src_frame == "map") {
      ex = src_x;
      ey = src_y;
      return true;
    }

    // 通过 TF2 查找 gimbal_yaw → map 的变换
    try {
      auto transform = tf_buffer_->lookupTransform(
        "map", src_frame,
        tf2::TimePointZero,  // 使用最新可用变换
        100ms);              // 超时 100ms，避免阻塞

      // 提取平移部分
      double tx = transform.transform.translation.x;
      double ty = transform.transform.translation.y;

      // 从四元数提取 yaw 角（2D 旋转只需要 z 分量和 w 分量）
      double qz = transform.transform.rotation.z;
      double qw = transform.transform.rotation.w;
      double yaw = 2.0 * std::atan2(qz, qw);

      // 2D 旋转矩阵 + 平移：map_pos = R(yaw) * src_pos + t
      double cos_yaw = std::cos(yaw);
      double sin_yaw = std::sin(yaw);
      ex = cos_yaw * src_x - sin_yaw * src_y + tx;
      ey = sin_yaw * src_x + cos_yaw * src_y + ty;
      return true;
    } catch (const tf2::TransformException & e) {
      // TF 变换失败：使用原始坐标作为近似值，打印警告
      RCLCPP_WARN(get_logger(),
        "敌人位置 TF 变换失败 (%s -> map): %s，使用原始坐标",
        src_frame.c_str(), e.what());
      ex = src_x;
      ey = src_y;
      return true;  // 变换失败但仍有位置数据，返回 true 让决策继续
    }
  }

  /**
   * @brief 计算到敌人的距离
   * target.position 在 gimbal_yaw 坐标系下，就是敌人相对于云台的相对位置
   * 直接用 sqrt(x² + y²) 即可得到距离，不需要 TF2 变换
   */
  double getEnemyDistance() const
  {
    if (!isEnemyVisible() || target_.msg.armors_num == 0) {return 999.0;}
    double dx = target_.msg.position.x;
    double dy = target_.msg.position.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  /**
   * @brief 判断机器人是否在指定类型的区域中
   * @param type 区域类型，如 "forbidden", "danger", "supply", "patrol" 等
   * 遍历所有匹配到的区域类型，看是否有匹配的
   */
  bool isInAreaType(const std::string & type) const
  {
    if (!area_status_.valid) {return false;}
    const auto & types = area_status_.msg.matched_area_types;
    return std::find(types.begin(), types.end(), type) != types.end();
  }

  /**
   * @brief 获取机器人在 map 坐标系下的位置
   * 1. 从里程计获取位置（通常在 odom 坐标系）
   * 2. 通过 tf2 将 odom -> map 转换
   * 3. 转换失败时使用上一次有效位置
   *
   * @param[out] rx, ry 机器人在 map 坐标系下的 x, y 坐标
   * @return true 表示位置有效
   */
  bool getRobotPosition(double & rx, double & ry)
  {
    if (!odom_.valid) {
      // 没有里程计数据，使用上一次位置
      if (has_last_position_) {
        rx = last_robot_x_;
        ry = last_robot_y_;
        return true;
      }
      return false;
    }

    // 尝试从里程计的 frame_id（通常是 odom）转换到 map
    const std::string & source_frame = odom_.msg.header.frame_id;
    const std::string & target_frame = "map";

    // 如果里程计已经在 map 坐标系下，直接使用
    if (source_frame.empty() || source_frame == target_frame) {
      rx = odom_.msg.pose.pose.position.x;
      ry = odom_.msg.pose.pose.position.y;
      last_robot_x_ = rx;
      last_robot_y_ = ry;
      has_last_position_ = true;
      return true;
    }

    // 通过 tf2 转换 odom -> map
    try {
      // 获取 odom 时刻的变换
      auto transform = tf_buffer_->lookupTransform(
        target_frame, source_frame,
        tf2::TimePointZero,  // 使用最新的变换
        100ms);              // 超时 100ms

      // 提取里程计中的位置
      geometry_msgs::msg::PointStamped odom_point;
      odom_point.header = odom_.msg.header;
      odom_point.point = odom_.msg.pose.pose.position;

      // 执行坐标变换
      geometry_msgs::msg::PointStamped map_point;
      tf2::doTransform(odom_point, map_point, transform);

      rx = map_point.point.x;
      ry = map_point.point.y;
      last_robot_x_ = rx;
      last_robot_y_ = ry;
      has_last_position_ = true;
      return true;
    } catch (const tf2::TransformException & ex) {
      // 转换失败：打印警告（节流，每 5 秒一次）
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF 变换失败 (%s -> %s): %s，使用上一次有效位置",
        source_frame.c_str(), target_frame.c_str(), ex.what());

      // 使用上一次有效位置
      if (has_last_position_) {
        rx = last_robot_x_;
        ry = last_robot_y_;
        return true;
      }
      return false;
    }
  }

  /**
   * @brief 计算在 danger_damage_window_ 时间窗口内的总掉血量
   * 遍历 damage_history_，累加在窗口内的掉血量
   * 同时清理过期的历史记录
   */
  double getRecentDamage()
  {
    rclcpp::Time current_time = now();
    double total_damage = 0.0;

    // 从后往前遍历，累加窗口内的伤害
    // 同时删除过期的记录
    auto it = damage_history_.begin();
    while (it != damage_history_.end()) {
      double age = (current_time - it->first).seconds();
      if (age > danger_damage_window_) {
        // 这条记录太旧了，删除
        it = damage_history_.erase(it);
      } else {
        total_damage += it->second;
        ++it;
      }
    }

    return total_damage;
  }

  /**
   * @brief 判断敌人是否在禁区内
   * 只有启用了敌人禁区订阅且数据有效时才返回 true
   */
  bool isEnemyInForbidden() const
  {
    if (!enable_enemy_forbidden_ || !enemy_forbidden_.valid) {return false;}
    return enemy_forbidden_.msg.is_forbidden;
  }

  /**
   * @brief 创建 PoseStamped 消息（map 坐标系）
   * @param x, y 目标位置的 x, y 坐标
   */
  geometry_msgs::msg::PoseStamped makePoseStamped(double x, double y) const
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = now();
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    // 四元数默认值（无旋转）
    ps.pose.orientation.w = 1.0;
    return ps;
  }

  // ====================================================================
  // 核心：计算各模式评分
  // ====================================================================

  /**
   * @brief 计算攻击评分
   *
   * 评分逻辑：
   * - 看不到敌人 → 0 分（不打）
   * - 敌人可见 +50
   * - 敌人在最佳攻击距离内 +20
   * - 血量健康 (>0.5) +20
   * - 血量低 (<0.35) -40（打不动了）
   * - 敌人在禁区 -100（禁止攻击禁区内的敌人）
   * - 自己在危险区域 -20
   */
  double computeAttackScore(double hp_ratio,
                            bool enemy_visible, std::string & reason) const
  {
    double score = 0.0;

    // 看不到敌人 → 攻击评分为 0
    if (!enemy_visible) {
      reason = "no enemy visible";
      return 0.0;
    }

    score += 50.0;
    reason = "enemy visible";

    // 计算到敌人的距离
    double enemy_dist = getEnemyDistance();
    if (enemy_dist >= attack_distance_min_ && enemy_dist <= attack_distance_max_) {
      score += 20.0;
      reason += ", in attack range";
    }

    // 血量判断
    if (hp_ratio > 0.5) {
      score += 20.0;
      reason += ", hp healthy";
    } else if (hp_ratio < low_hp_ratio_) {
      score -= 40.0;
      reason += ", low hp";
    }

    // 敌人在禁区 → 大幅降低攻击分
    if (isEnemyInForbidden()) {
      score -= 100.0;
      reason += ", enemy in forbidden";
    }

    // 自己在危险区域
    if (isInAreaType("forbidden") || isInAreaType("danger")) {
      score -= 20.0;
      reason += ", self in danger zone";
    }

    return score;
  }

  /**
   * @brief 计算撤退评分
   *
   * 评分逻辑：
   * - 低血量 (<0.35) +50
   * - 极低血量 (<0.20) +40（叠加）
   * - 短时间内大量掉血 +40
   * - 自己在危险区域 +30
   * - 被敌人近距离锁定 (<2.0m) +20
   */
  double computeRetreatScore(double hp_ratio,
                             bool enemy_visible, std::string & reason) const
  {
    double score = 0.0;
    reason = "";

    // 低血量判断
    if (hp_ratio < low_hp_ratio_) {
      score += 50.0;
      reason += "low hp";
    }

    // 极低血量（叠加低血量加分）
    if (hp_ratio < critical_hp_ratio_) {
      score += 40.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "critical hp";
    }

    // 近期掉血过多
    // 注意：getRecentDamage() 会清理过期记录，所以这里不能在 const 函数里调用
    // 我会在 computeDecision() 里计算好传入
    // 这里先用占位符

    // 自己在危险区域
    if (isInAreaType("forbidden") || isInAreaType("danger")) {
      score += 30.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "in danger zone";
    }

    // 被敌人近距离锁定
    if (enemy_visible) {
      double enemy_dist = getEnemyDistance();
      if (enemy_dist < 2.0) {
        score += 20.0;
        if (!reason.empty()) {reason += ", ";}
        reason += "enemy too close";
      }
    }

    if (reason.empty()) {
      reason = "no retreat reason";
    }

    return score;
  }

  /**
   * @brief 计算补给评分
   *
   * 评分逻辑：
   * - 低血量 (<0.35) +40
   * - 弹药不足或热量接近上限 +20
   * - 已经在补给区 +20
   * - 血量健康 (>0.8) 降低 30（不需要补给）
   */
  double computeSupplyScore(double hp_ratio, std::string & reason) const
  {
    double score = 0.0;
    reason = "";

    // 低血量
    if (hp_ratio < low_hp_ratio_) {
      score += 40.0;
      reason = "low hp";
    }

    // 弹药不足或热量过高
    double heat_ratio = getHeatRatio();
    if (heat_ratio > 0.8) {
      score += 20.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "heat high";
    }
    // 弹丸余量检查（shot_allowance）
    if (robot_status_.valid && robot_status_.msg.shot_allowance < 50) {
      score += 20.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "low ammo";
    }

    // 已经在补给区
    if (isInAreaType("supply")) {
      score += 20.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "in supply zone";
    }

    // 血量健康则降低补给优先级
    if (hp_ratio > 0.8) {
      score -= 30.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "hp healthy";
    }

    if (reason.empty()) {
      reason = "no supply need";
    }

    return score;
  }

  /**
   * @brief 计算巡逻评分
   *
   * 评分逻辑：
   * - 没看到敌人 +40（闲着也是闲着，去巡逻）
   * - 血量尚可 (>0.35) +20
   * - 已经在巡逻区 +10
   */
  double computePatrolScore(double hp_ratio, bool enemy_visible,
                            std::string & reason) const
  {
    double score = 0.0;
    reason = "";

    // 没有敌人 → 巡逻
    if (!enemy_visible) {
      score += 40.0;
      reason = "no enemy";
    }

    // 血量尚可
    if (hp_ratio > low_hp_ratio_) {
      score += 20.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "hp ok";
    }

    // 在 robot_area（如起伏路段）时强制巡逻（移动姿态）
    if (isInAreaType(robot_area_)) {
      score += 100.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "in " + robot_area_ + ", force patrol";
    }

    // 已经在巡逻区
    if (isInAreaType("patrol")) {
      score += 10.0;
      if (!reason.empty()) {reason += ", ";}
      reason += "in patrol zone";
    }

    if (reason.empty()) {
      reason = "enemy visible, skip patrol";
    }

    return score;
  }

  /**
   * @brief 计算防守基地评分
   *
   * 评分逻辑（第一版）：
   * - TODO: 敌人接近己方基地 +60（需要基地位置信息）
   * - TODO: 己方基地血量低 +40（需要基地血量信息）
   * - 第一版保留接口，默认评分为 0
   */
  double computeDefendBaseScore(std::string & reason) const
  {
    double score = 0.0;
    reason = "defend base (TODO: need base info)";

    // TODO: 以下是未来版本的接口，当前保留
    // - 如果敌人位置接近己方基地坐标 → score += 60.0
    // - 如果己方基地血量低于阈值 → score += 40.0
    // 需要额外的话题订阅（基地血量、基地位置）

    return score;
  }

  // ====================================================================
  // 核心决策函数
  // ====================================================================

  /**
   * @brief 计算决策结果（定时器回调，10Hz）
   *
   * 工作流程：
   * 1. 检查比赛状态 → 不在比赛中就发 IDLE
   * 2. 获取机器人位置
   * 3. 计算所有模式评分
   * 4. 选择最高分模式
   * 5. 滞回防抖判断
   * 6. 发布决策结果
   */
  void computeDecision()
  {
    auto decision = rm_decision_interfaces::msg::SentryDecision();
    decision.header.frame_id = "map";
    decision.header.stamp = now();

    // ------------------------------------------------------------------
    // 第一步：检查比赛状态
    // 只有 game_progress == 4 时才做战术决策
    // ------------------------------------------------------------------
    if (!game_status_.valid ||
        game_status_.msg.game_progress != 4)
    {
      decision.mode = MODE_IDLE;
      decision.attack_score = 0.0;
      decision.retreat_score = 0.0;
      decision.supply_score = 0.0;
      decision.patrol_score = 0.0;
      decision.defend_base_score = 0.0;
      decision.reason = "not in game";
      decision.target_pose = makePoseStamped(
        has_last_position_ ? last_robot_x_ : 0.0,
        has_last_position_ ? last_robot_y_ : 0.0);

      // 比赛未开始时也更新状态
      current_mode_ = MODE_IDLE;
      current_score_ = 0.0;
      decision_pub_->publish(decision);
      return;
    }

    // ------------------------------------------------------------------
    // 第二步：获取机器人当前位置
    // ------------------------------------------------------------------
    double robot_x = 0.0, robot_y = 0.0;
    bool has_position = getRobotPosition(robot_x, robot_y);

    if (!has_position) {
      // 没有位置信息，无法做决策，发布 IDLE
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "无有效位置数据，决策为 IDLE");
      decision.mode = MODE_IDLE;
      decision.reason = "no position data";
      decision.target_pose = makePoseStamped(0.0, 0.0);
      decision_pub_->publish(decision);
      return;
    }

    // ------------------------------------------------------------------
    // 第三步：收集决策所需的中间状态
    // ------------------------------------------------------------------
    double hp_ratio = getHpRatio();                    // 血量比例
    bool   enemy_visible = isEnemyVisible();           // 是否看到敌人
    double recent_damage = getRecentDamage();           // 近期掉血量

    // ------------------------------------------------------------------
    // 第四步：计算各模式评分
    // ------------------------------------------------------------------
    std::string attack_reason, retreat_reason, supply_reason;
    std::string patrol_reason, defend_reason;

    double attack_score      = computeAttackScore(
      hp_ratio, enemy_visible, attack_reason);

    double retreat_score     = computeRetreatScore(
      hp_ratio, enemy_visible, retreat_reason);

    // 加上近期掉血的额外撤退分
    if (recent_damage > damage_retreat_threshold_) {
      retreat_score += 40.0;
      retreat_reason += ", high damage rate";
    }

    double supply_score      = computeSupplyScore(hp_ratio, supply_reason);
    double patrol_score      = computePatrolScore(hp_ratio, enemy_visible, patrol_reason);
    double defend_base_score = computeDefendBaseScore(defend_reason);

    // 填充评分到消息
    decision.attack_score      = attack_score;
    decision.retreat_score     = retreat_score;
    decision.supply_score      = supply_score;
    decision.patrol_score      = patrol_score;
    decision.defend_base_score = defend_base_score;

    // ------------------------------------------------------------------
    // 第五步：选择最高分的模式
    // ------------------------------------------------------------------
    struct Candidate {
      std::string mode;
      double      score;
      std::string reason;
    };

    // 把所有候选模式放入列表
    std::vector<Candidate> candidates = {
      {MODE_ATTACK,      attack_score,      attack_reason},
      {MODE_RETREAT,     retreat_score,     retreat_reason},
      {MODE_SUPPLY,      supply_score,      supply_reason},
      {MODE_PATROL,      patrol_score,      patrol_reason},
      {MODE_DEFEND_BASE, defend_base_score, defend_reason},
    };

    // 找到评分最高的候选
    auto best = std::max_element(candidates.begin(), candidates.end(),
      [](const Candidate & a, const Candidate & b) {
        return a.score < b.score;
      });

    std::string new_mode  = best->mode;
    double      new_score = best->score;
    std::string new_reason = best->reason;

    // ------------------------------------------------------------------
    // 第六步：滞回防抖 + 强制规则
    // ------------------------------------------------------------------

    // 强制规则 1：极低血量必须撤退
    if (hp_ratio < critical_hp_ratio_) {
      new_mode = MODE_RETREAT;
      new_reason = "critical hp, force retreat";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "极低血量 (%.1f%%)，强制撤退", hp_ratio * 100.0);
    }

    // 强制规则 2：敌人在禁区时禁止攻击
    if (new_mode == MODE_ATTACK && isEnemyInForbidden()) {
      new_mode = MODE_PATROL;
      new_reason = "enemy in forbidden area, attack disabled";
    }

    // 滞回防抖：
    // 如果当前模式不是 IDLE，且新模式分数没有超过当前分数 + switch_margin_，
    // 则保持当前模式不变（防止模式频繁跳变）
    if (current_mode_ != MODE_IDLE &&
        current_mode_ != new_mode &&
        new_score <= current_score_ + switch_margin_) {
      // 保持当前模式
      new_mode = current_mode_;
      new_reason = "hysteresis: keeping " + current_mode_;
      // 但要重新找到对应分数
      for (const auto & c : candidates) {
        if (c.mode == new_mode) {
          new_score = c.score;
          break;
        }
      }
    }

    // ------------------------------------------------------------------
    // 第七步：设置目标点
    // ------------------------------------------------------------------
    if (new_mode == MODE_ATTACK) {
      // 攻击模式：目标点为敌人位置
      double ex = 0.0, ey = 0.0;
      if (extractEnemyTarget(ex, ey)) {
        decision.target_pose = makePoseStamped(ex, ey);
      } else {
        decision.target_pose = makePoseStamped(robot_x, robot_y);
      }
    } else if (new_mode == MODE_RETREAT) {
      decision.target_pose = makePoseStamped(retreat_goal_x_, retreat_goal_y_);
    } else if (new_mode == MODE_SUPPLY) {
      decision.target_pose = makePoseStamped(supply_goal_x_, supply_goal_y_);
    } else if (new_mode == MODE_DEFEND_BASE) {
      decision.target_pose = makePoseStamped(defend_goal_x_, defend_goal_y_);
    } else if (new_mode == MODE_PATROL) {
      decision.target_pose = makePoseStamped(patrol_goal_x_, patrol_goal_y_);
    } else {
      // IDLE：填当前位置
      decision.target_pose = makePoseStamped(robot_x, robot_y);
    }

    // ------------------------------------------------------------------
    // 第八步：发布决策
    // ------------------------------------------------------------------
    decision.mode   = new_mode;
    decision.reason = new_reason;

    decision_pub_->publish(decision);

    // 更新当前模式状态
    current_mode_  = new_mode;
    current_score_ = new_score;
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(UtilityAiNode)
