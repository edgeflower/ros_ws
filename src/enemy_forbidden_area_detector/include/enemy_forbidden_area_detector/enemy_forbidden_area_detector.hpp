#ifndef ENEMY_FORBIDDEN_AREA_DETECTOR__DETECTOR_HPP_
#define ENEMY_FORBIDDEN_AREA_DETECTOR__DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

#include "armor_interfaces/msg/target.hpp"
#include "enemy_forbidden_area_detector/msg/enemy_forbidden_area.hpp"
#include "enemy_forbidden_area_detector/geometry_utils.hpp"
#include "enemy_forbidden_area_detector/marker_utils.hpp"

namespace enemy_forbidden_area_detector {

/**
 * @class EnemyForbiddenAreaDetector
 * @brief 检测敌人是否进入 forbidden 禁区的 ROS2 节点
 *
 * 工作流程：
 * ========
 * 1. 启动时从 areas.yaml 加载所有 type == forbidden 的多边形
 * 2. 订阅 /target_tracking 获取敌方位置
 * 3. 使用 TF2 将敌方位置转换到 map 坐标系
 * 4. 使用射线法判断敌人是否在 forbidden 多边形内
 * 5. 通过连续检测滤波消除边缘抖动
 * 6. 发布 EnemyForbiddenArea 消息
 * 7. 发布 RViz 可视化 Marker
 *
 * 为什么必须统一使用 map 坐标系？
 * ================================
 * - forbidden 禁区是在地图上定义的绝对位置约束
 * - odom 坐标系以机器人初始位置为原点，会随时间漂移
 * - 如果用 odom 定义禁区，禁区会跟着机器人"跑"，失去绝对意义
 * - 只有 map 坐标系能提供固定不变的绝对参考
 * - 无论 SLAM 在线还是离线地图，禁区定义都基于 map 坐标系
 */
class EnemyForbiddenAreaDetector : public rclcpp::Node {
 public:
  explicit EnemyForbiddenAreaDetector(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  // ==================== ROS 通信 ====================

  /// 订阅敌方目标信息
  rclcpp::Subscription<armor_interfaces::msg::Target>::SharedPtr target_sub_;

  /// 发布禁区检测结果
  rclcpp::Publisher<msg::EnemyForbiddenArea>::SharedPtr result_pub_;

  /// 可视化发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr forbidden_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr enemy_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_marker_pub_;

  // ==================== TF2 ====================

  /**
   * TF2 用于坐标变换：
   * - /target_tracking 的敌方位置在 gimbal_yaw 坐标系
   * - 禁区在 map 坐标系
   * - 必须通过 TF2 将 gimbal_yaw → map 才能做位置比较
   *
   * tf2_ros::Buffer: 存储 TF 树数据，提供 lookupTransform 查询
   * tf2_ros::TransformListener: 订阅 /tf 和 /tf_static，自动更新 Buffer
   */
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ==================== 滤波状态 ====================

  /// 连续检测为"在禁区内"的次数计数器
  int inside_counter_ = 0;

  /// 连续检测为"在禁区外"的次数计数器
  int outside_counter_ = 0;

  /// 当前滤波后的输出状态（经连续确认后的稳定状态）
  bool filtered_is_forbidden_ = false;

  /// 上一次发布的 armors_num（用于无目标检测）
  int32_t last_armors_num_ = 0;

  // ==================== 检测参数 ====================

  int inside_threshold_;
  int outside_threshold_;
  double transform_timeout_;
  std::string map_frame_;
  bool enable_visualization_;

  // ==================== 数据 ====================

  /// 所有 forbidden 多边形
  std::vector<ForbiddenPolygon> forbidden_polygons_;

  // ==================== 回调函数 ====================

  /**
   * @brief /target_tracking 回调
   *
   * 每次收到敌方目标信息时触发，执行完整的检测流程
   */
  void onTargetTracking(const armor_interfaces::msg::Target::SharedPtr msg);

  // ==================== 内部方法 ====================

  /**
   * @brief 从 YAML 文件加载 forbidden 多边形
   *
   * 只加载 type == "forbidden" 的多边形，忽略其他类型
   */
  bool loadForbiddenPolygons(const std::string& yaml_path);

  /**
   * @brief 将敌方位置从源坐标系转换到 map 坐标系
   *
   * 使用 TF2 的 lookupTransform 进行坐标变换。
   * 如果 TF 失败（超时/坐标系不存在），返回 false 并保持安全状态。
   *
   * 为什么 TF 失败时不能崩溃？
   * - 机器人启动阶段 TF 树可能还没建立完整
   * - SLAM 定位偶尔会丢失，导致 TF 暂时不可用
   * - 作为检测节点，必须比其他节点更健壮
   */
  bool transformToMap(
      const std::string& source_frame,
      double src_x, double src_y,
      double& map_x, double& map_y,
      const rclcpp::Time& stamp);

  /**
   * @brief 连续检测滤波
   *
   * 为什么 polygon 边缘容易抖动？
   * ================================
   * 1. TF 变换有误差（几厘米级别）
   * 2. SLAM 定位会周期性修正（产生位置跳变）
   * 3. 浮点运算有精度损失
   * 4. 目标检测的位置本身有噪声
   *
   * 当敌人刚好在禁区边缘时，以上误差会导致
   * 检测结果在 true/false 之间高频切换，触发误报。
   *
   * 滤波方案：
   * - 连续 inside_threshold 次检测为 inside → 确认进入禁区
   * - 连续 outside_threshold 次检测为 outside → 确认离开禁区
   * - outside_threshold 默认比 inside_threshold 大（离开需要更确定的信号）
   */
  bool applyDebounceFilter(bool raw_inside);

  /**
   * @brief 发布检测结果
   */
  void publishResult(int32_t armors_num, bool is_forbidden);

  /**
   * @brief 发布可视化 Marker
   */
  void publishVisualization(
      double enemy_x, double enemy_y,
      bool is_forbidden, bool has_target);
};

}  // namespace enemy_forbidden_area_detector

#endif  // ENEMY_FORBIDDEN_AREA_DETECTOR__DETECTOR_HPP_
