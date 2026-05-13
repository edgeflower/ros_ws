#include "enemy_forbidden_area_detector/enemy_forbidden_area_detector.hpp"
#include <tf2/exceptions.h>
#include <filesystem>

namespace enemy_forbidden_area_detector {

EnemyForbiddenAreaDetector::EnemyForbiddenAreaDetector(
    const rclcpp::NodeOptions& options)
    : Node("enemy_forbidden_area_detector", options) {
  // ==================== 声明并获取参数 ====================
  this->declare_parameter("areas_yaml_path",
      "/home/lu/ros_ws/src/polygon_manager/config/areas.yaml");
  this->declare_parameter("target_topic", "/target_tracking");
  this->declare_parameter("result_topic", "/enemy_in_forbidden_area");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("inside_threshold", 5);
  this->declare_parameter("outside_threshold", 10);
  this->declare_parameter("transform_timeout", 0.2);
  this->declare_parameter("enable_visualization", true);

  std::string areas_yaml_path = this->get_parameter("areas_yaml_path").as_string();
  std::string target_topic = this->get_parameter("target_topic").as_string();
  std::string result_topic = this->get_parameter("result_topic").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  inside_threshold_ = this->get_parameter("inside_threshold").as_int();
  outside_threshold_ = this->get_parameter("outside_threshold").as_int();
  transform_timeout_ = this->get_parameter("transform_timeout").as_double();
  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();

  // ==================== 加载 forbidden 多边形 ====================
  if (!loadForbiddenPolygons(areas_yaml_path)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to load forbidden polygons from: %s",
                 areas_yaml_path.c_str());
  }

  // ==================== 初始化 TF2 ====================
  // tf_buffer_ 存储 TF 树，tf_listener_ 自动订阅 /tf 和 /tf_static 更新 buffer
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ==================== 创建发布者 ====================
  result_pub_ = this->create_publisher<msg::EnemyForbiddenArea>(
      result_topic, 10);

  if (enable_visualization_) {
    forbidden_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/forbidden_area_marker", 10);
    enemy_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/enemy_position_marker", 10);
    text_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/enemy_forbidden_text_marker", 10);

    // 启动时发布一次 forbidden 区域可视化
    auto markers = MarkerUtils::createForbiddenAreaMarkers(
        forbidden_polygons_, this->now());
    forbidden_marker_pub_->publish(markers);
  }

  // ==================== 创建订阅者 ====================
  target_sub_ = this->create_subscription<armor_interfaces::msg::Target>(
      target_topic, 10,
      std::bind(&EnemyForbiddenAreaDetector::onTargetTracking, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "EnemyForbiddenAreaDetector started. "
              "Loaded %zu forbidden polygons, inside_threshold=%d, outside_threshold=%d",
              forbidden_polygons_.size(), inside_threshold_, outside_threshold_);
}

bool EnemyForbiddenAreaDetector::loadForbiddenPolygons(
    const std::string& yaml_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    if (!config["areas"]) {
      RCLCPP_WARN(this->get_logger(), "No 'areas' field in YAML: %s", yaml_path.c_str());
      return false;
    }

    for (const auto& area_node : config["areas"]) {
      std::string type_str = area_node["type"].as<std::string>();

      // 只加载 forbidden 类型，忽略 protect/patrol/attack
      if (type_str != "forbidden") continue;

      ForbiddenPolygon polygon;
      polygon.name = area_node["name"].as<std::string>();

      for (const auto& point_node : area_node["points"]) {
        geometry_msgs::msg::Point p;
        p.x = point_node[0].as<double>();
        p.y = point_node[1].as<double>();
        p.z = 0.0;
        polygon.vertices.push_back(p);
      }

      if (GeometryUtils::isValidPolygon(polygon.vertices)) {
        forbidden_polygons_.push_back(polygon);
        RCLCPP_INFO(this->get_logger(),
                    "Loaded forbidden polygon '%s' (%zu vertices)",
                    polygon.name.c_str(), polygon.vertices.size());
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Skipping invalid polygon '%s'", polygon.name.c_str());
      }
    }

    return !forbidden_polygons_.empty();

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to parse YAML '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }
}

void EnemyForbiddenAreaDetector::onTargetTracking(
    const armor_interfaces::msg::Target::SharedPtr msg) {
  // ==================== 步骤 1：判断是否有目标 ====================
  // armors_num == 0 表示无目标，直接发布安全状态
  int32_t armors_num = msg->armors_num;

  if (armors_num == 0 || !msg->tracking) {
    // 无目标：重置滤波状态，发布安全状态
    inside_counter_ = 0;
    outside_counter_ = 0;
    filtered_is_forbidden_ = false;
    publishResult(0, false);

    if (enable_visualization_) {
      publishVisualization(0, 0, false, false);
    }
    return;
  }

  // ==================== 步骤 2：提取敌方位置并转换到 map 坐标系 ====================
  double enemy_map_x = 0.0, enemy_map_y = 0.0;

  // target_tracking 的 frame_id 通常是 "gimbal_yaw" 或其他非 map 坐标系
  // 必须通过 TF2 转换到 map 坐标系才能与禁区（map 坐标系定义）做比较
  if (!transformToMap(msg->header.frame_id,
                      msg->position.x, msg->position.y,
                      enemy_map_x, enemy_map_y,
                      msg->header.stamp)) {
    // TF 转换失败：保持上一次稳定状态，不更新
    // 这样做比发布 false 更安全，避免因临时 TF 抖动产生误报
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "TF transform failed, keeping last state");
    publishResult(armors_num, filtered_is_forbidden_);
    return;
  }

  // ==================== 步骤 3：遍历所有 forbidden polygon，检测敌人是否在内 ====================
  bool raw_inside = false;
  for (const auto& polygon : forbidden_polygons_) {
    if (GeometryUtils::isPointInPolygon(enemy_map_x, enemy_map_y,
                                         polygon.vertices)) {
      raw_inside = true;
      break;  // 进入任意一个 forbidden polygon 即可
    }
  }

  // ==================== 步骤 4：连续检测滤波 ====================
  bool filtered = applyDebounceFilter(raw_inside);

  // ==================== 步骤 5：发布结果 ====================
  publishResult(armors_num, filtered);

  if (enable_visualization_) {
    publishVisualization(enemy_map_x, enemy_map_y, filtered, true);
  }
}

bool EnemyForbiddenAreaDetector::transformToMap(
    const std::string& source_frame,
    double src_x, double src_y,
    double& map_x, double& map_y,
    const rclcpp::Time& stamp) {
  try {
    // 查询从 source_frame 到 map_frame 的坐标变换
    // transform_timeout_ 设置超时时间，避免长时间阻塞
    auto transform = tf_buffer_->lookupTransform(
        map_frame_, source_frame, stamp,
        tf2::durationFromSec(transform_timeout_));

    // 应用坐标变换：旋转 + 平移
    // 对于 2D 点（x, y），只需要取变换的平移部分和 yaw 旋转
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;

    // 从四元数提取 yaw 角
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;
    double yaw = 2.0 * std::atan2(qz, qw);

    // 旋转矩阵 2D：[cos(yaw), -sin(yaw); sin(yaw), cos(yaw)]
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // 变换点：(map_x, map_y) = R * (src_x, src_y) + (tx, ty)
    map_x = cos_yaw * src_x - sin_yaw * src_y + tx;
    map_y = sin_yaw * src_x + cos_yaw * src_y + ty;

    return true;

  } catch (const tf2::TransformException& e) {
    // TF 失败的常见原因：
    // 1. 坐标系还没建立（启动初期）
    // 2. SLAM 丢失导致 TF 断裂
    // 3. 超时（网络延迟或计算延迟）
    // 无论哪种原因，都不应该让节点崩溃
    RCLCPP_DEBUG(this->get_logger(),
                 "TF failed: %s → %s: %s",
                 source_frame.c_str(), map_frame_.c_str(), e.what());
    return false;
  }
}

bool EnemyForbiddenAreaDetector::applyDebounceFilter(bool raw_inside) {
  if (raw_inside) {
    // 检测为"在禁区内"
    inside_counter_++;
    outside_counter_ = 0;  // 重置 outside 计数器

    // 连续 inside_threshold_ 次检测为 inside → 确认进入
    if (inside_counter_ >= inside_threshold_) {
      filtered_is_forbidden_ = true;
      // 不清零 inside_counter_，保持状态直到 outside_threshold 确认离开
    }
  } else {
    // 检测为"在禁区外"
    outside_counter_++;
    inside_counter_ = 0;  // 重置 inside 计数器

    // 连续 outside_threshold_ 次检测为 outside → 确认离开
    if (outside_counter_ >= outside_threshold_) {
      filtered_is_forbidden_ = false;
    }
  }

  return filtered_is_forbidden_;
}

void EnemyForbiddenAreaDetector::publishResult(
    int32_t armors_num, bool is_forbidden) {
  auto msg = msg::EnemyForbiddenArea();
  msg.armors_num = armors_num;
  msg.is_forbidden = is_forbidden;
  result_pub_->publish(msg);
}

void EnemyForbiddenAreaDetector::publishVisualization(
    double enemy_x, double enemy_y,
    bool is_forbidden, bool has_target) {
  auto now = this->now();

  // forbidden 区域边界（红色线条）
  auto forbidden_markers = MarkerUtils::createForbiddenAreaMarkers(
      forbidden_polygons_, now);
  forbidden_marker_pub_->publish(forbidden_markers);

  // 敌方位置球体
  auto enemy_marker = MarkerUtils::createEnemyPositionMarker(
      enemy_x, enemy_y, is_forbidden, has_target, now);
  enemy_marker_pub_->publish(enemy_marker);

  // 状态文字
  auto text_marker = MarkerUtils::createStatusTextMarker(
      enemy_x, enemy_y, is_forbidden, has_target, now);
  text_marker_pub_->publish(text_marker);
}

}  // namespace enemy_forbidden_area_detector
