#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>

#include "robot_area_detector/msg/robot_area_status.hpp"
#include <rm_decision_interfaces/msg/robot_area_status.hpp>

namespace robot_area_detector {

// ==================== 数据结构 ====================

struct Point2D {
  double x;
  double y;
};

struct Area {
  std::string name;
  std::string type;
  std::vector<Point2D> points;
};

// ==================== 几何算法 ====================

// 浮点精度常量
static constexpr double EPSILON = 1e-9;

/**
 * 判断点 (px, py) 是否在线段 (a, b) 上
 * 用于边界情况处理：点恰好在多边形边上时也算"在内部"
 */
bool pointOnSegment(double px, double py, const Point2D& a, const Point2D& b) {
  // 先检查点是否在线段的包围盒内
  double min_x = std::min(a.x, b.x) - EPSILON;
  double max_x = std::max(a.x, b.x) + EPSILON;
  double min_y = std::min(a.y, b.y) - EPSILON;
  double max_y = std::max(a.y, b.y) + EPSILON;

  if (px < min_x || px > max_x || py < min_y || py > max_y) {
    return false;
  }

  // 用叉积判断点是否在线段的直线上
  // 叉积 = 0 表示三点共线
  double cross = (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x);
  return std::abs(cross) < EPSILON;
}

/**
 * 射线法判断点是否在多边形内
 *
 * 算法原理：
 * 从点 (x, y) 向右（x 正方向）发射一条水平射线，
 * 遍历多边形的每条边，统计射线与边的交点数：
 *   奇数个交点 → 点在内部
 *   偶数个交点 → 点在外部
 *
 * 支持任意多边形（凸/凹），O(n) 时间复杂度
 */
bool pointInPolygon(double x, double y, const std::vector<Point2D>& polygon) {
  int n = static_cast<int>(polygon.size());
  if (n < 3) return false;

  bool inside = false;

  // 先检查边界：点是否在任何一条边上
  for (int i = 0, j = n - 1; i < n; j = i++) {
    if (pointOnSegment(x, y, polygon[j], polygon[i])) {
      return true;
    }
  }

  // 射线法核心逻辑
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double yi = polygon[i].y, yj = polygon[j].y;
    double xi = polygon[i].x, xj = polygon[j].x;

    // y 是否在边的 y 范围内（不含上端点，避免顶点重复计数）
    if (((yi > y) != (yj > y))) {
      double x_intersect = xj + (y - yj) / (yi - yj) * (xi - xj);
      if (x < x_intersect) {
        inside = !inside;
      }
    }
  }

  return inside;
}

// ==================== YAML 解析 ====================

/**
 * 解析 areas.yaml 文件
 *
 * 兼容两种格式：
 * 格式一（polygon_manager 输出）：
 *   areas:
 *     - name: xxx
 *       type: forbidden
 *       points:
 *         - [1.0, 2.0]
 *         - [3.0, 4.0]
 *
 * 格式二（字典格式）：
 *   areas:
 *     area_name:
 *       type: patrol
 *       points:
 *         - x: 1.0
 *           y: 2.0
 */
std::vector<Area> loadAreasFromYaml(const std::string& yaml_path) {
  std::vector<Area> areas;

  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    if (!config["areas"]) {
      return areas;
    }

    const YAML::Node& areas_node = config["areas"];

    if (areas_node.IsSequence()) {
      // 格式一：Sequence
      for (const auto& area_node : areas_node) {
        Area area;
        area.name = area_node["name"].as<std::string>();
        area.type = area_node["type"].as<std::string>();

        for (const auto& pt : area_node["points"]) {
          if (pt.IsSequence()) {
            // [x, y] 格式
            area.points.push_back({pt[0].as<double>(), pt[1].as<double>()});
          } else if (pt.IsMap()) {
            // {x: ..., y: ...} 格式
            area.points.push_back({pt["x"].as<double>(), pt["y"].as<double>()});
          }
        }

        areas.push_back(area);
      }
    } else if (areas_node.IsMap()) {
      // 格式二：Map（name 作为 key）
      for (const auto& entry : areas_node) {
        Area area;
        area.name = entry.first.as<std::string>();
        area.type = entry.second["type"].as<std::string>();

        for (const auto& pt : entry.second["points"]) {
          if (pt.IsSequence()) {
            area.points.push_back({pt[0].as<double>(), pt[1].as<double>()});
          } else if (pt.IsMap()) {
            area.points.push_back({pt["x"].as<double>(), pt["y"].as<double>()});
          }
        }

        areas.push_back(area);
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_area_detector"),
                 "Failed to load YAML '%s': %s", yaml_path.c_str(), e.what());
  }

  return areas;
}

// ==================== 可视化工具 ====================

visualization_msgs::msg::Marker createRobotPositionMarker(
    double x, double y, bool in_area, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = "robot_position";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.2;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // 在区域内 → 蓝色，不在 → 白色
  marker.color.r = in_area ? 0.0f : 1.0f;
  marker.color.g = in_area ? 0.5f : 1.0f;
  marker.color.b = in_area ? 1.0f : 1.0f;
  marker.color.a = 1.0f;

  return marker;
}

visualization_msgs::msg::MarkerArray createAreaMarkers(
    const std::vector<Area>& all_areas,
    const std::vector<std::string>& matched_names,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray array;

  for (size_t i = 0; i < all_areas.size(); ++i) {
    const auto& area = all_areas[i];
    if (area.points.size() < 3) continue;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "matched_area";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.04;
    marker.pose.orientation.w = 1.0;

    // 判断该区域是否命中
    bool matched = false;
    for (const auto& name : matched_names) {
      if (name == area.name) { matched = true; break; }
    }

    if (matched) {
      // 命中区域：亮青色
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0f;
    } else {
      // 未命中区域：暗灰色
      marker.color.r = 0.4f;
      marker.color.g = 0.4f;
      marker.color.b = 0.4f;
      marker.color.a = 0.4f;
    }

    // 添加顶点并闭环
    for (const auto& pt : area.points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    // 闭环
    geometry_msgs::msg::Point p;
    p.x = area.points[0].x;
    p.y = area.points[0].y;
    p.z = 0.0;
    marker.points.push_back(p);

    array.markers.push_back(marker);

    // 区域名称文字 Marker
    visualization_msgs::msg::Marker text;
    text.header.frame_id = frame_id;
    text.header.stamp = stamp;
    text.ns = "area_text";
    text.id = static_cast<int>(i);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    // 计算多边形中心点作为文字位置
    double cx = 0, cy = 0;
    for (const auto& pt : area.points) { cx += pt.x; cy += pt.y; }
    cx /= area.points.size();
    cy /= area.points.size();

    text.pose.position.x = cx;
    text.pose.position.y = cy;
    text.pose.position.z = 0.3;
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.3;
    text.text = area.name + " (" + area.type + ")";

    if (matched) {
      text.color.r = 0.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
    } else {
      text.color.r = 0.5f;
      text.color.g = 0.5f;
      text.color.b = 0.5f;
    }
    text.color.a = 1.0f;

    array.markers.push_back(text);
  }

  return array;
}

// ==================== 节点类 ====================

class RobotAreaDetectorNode : public rclcpp::Node {
 public:
  explicit RobotAreaDetectorNode(const rclcpp::NodeOptions& options)
      : Node("robot_area_detector_node", options) {
    // 声明参数
    this->declare_parameter("areas_yaml",
        "/home/lu/ros_ws/src/polygon_manager/config/areas.yaml");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("robot_frame", "chassis");
    this->declare_parameter("check_frequency", 20.0);
    this->declare_parameter("tf_timeout", 0.1);
    this->declare_parameter("status_topic", "/robot_area_status");
    this->declare_parameter("marker_topic", "/robot_area_detector/markers");
    this->declare_parameter("target_area_type", std::string(""));
    this->declare_parameter("target_area_name", std::string(""));
    this->declare_parameter("publish_marker", true);
    this->declare_parameter("print_debug", false);

    // 获取参数
    std::string areas_yaml = this->get_parameter("areas_yaml").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    double check_freq = this->get_parameter("check_frequency").as_double();
    tf_timeout_ = this->get_parameter("tf_timeout").as_double();
    target_area_type_ = this->get_parameter("target_area_type").as_string();
    target_area_name_ = this->get_parameter("target_area_name").as_string();
    publish_marker_ = this->get_parameter("publish_marker").as_bool();
    print_debug_ = this->get_parameter("print_debug").as_bool();

    // 加载区域
    areas_ = loadAreasFromYaml(areas_yaml);
    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu areas from '%s'",
                areas_.size(), areas_yaml.c_str());
    for (const auto& a : areas_) {
      RCLCPP_INFO(this->get_logger(),
                  "  [%s] %s (%zu pts)", a.type.c_str(), a.name.c_str(), a.points.size());
    }

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 发布者
    std::string status_topic = this->get_parameter("status_topic").as_string();
    std::string marker_topic = this->get_parameter("marker_topic").as_string();

    status_pub_ = this->create_publisher<rm_decision_interfaces::msg::RobotAreaStatus>(status_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic, 10);

    // 定时器
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / check_freq)),
        std::bind(&RobotAreaDetectorNode::checkCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "RobotAreaDetector started (%.1f Hz, %s -> %s)",
                check_freq, map_frame_.c_str(), robot_frame_.c_str());
  }

 private:
  void checkCallback() {
    auto now = this->now();

    // 查询 TF: map_frame -> robot_frame
    double robot_x = std::nan("");
    double robot_y = std::nan("");
    bool tf_ok = false;

    try {
      auto transform = tf_buffer_->lookupTransform(
          map_frame_, robot_frame_,
          tf2::TimePointZero,
          tf2::durationFromSec(tf_timeout_));

      robot_x = transform.transform.translation.x;
      robot_y = transform.transform.translation.y;
      tf_ok = true;
    } catch (const tf2::TransformException& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "TF %s→%s failed: %s",
                           map_frame_.c_str(), robot_frame_.c_str(), e.what());
    }

    // 构建消息
    rm_decision_interfaces::msg::RobotAreaStatus status;
    status.header.stamp = now;
    status.header.frame_id = map_frame_;
    status.robot_x = robot_x;
    status.robot_y = robot_y;
    status.is_in_area = false;
    status.area_name = "";
    status.area_type = "";
    status.matched_area_count = 0;

    if (tf_ok) {
      // 遍历所有区域，检测
      for (const auto& area : areas_) {
        // 按类型过滤
        if (!target_area_type_.empty() && area.type != target_area_type_) continue;
        // 按名称过滤
        if (!target_area_name_.empty() && area.name != target_area_name_) continue;

        if (pointInPolygon(robot_x, robot_y, area.points)) {
          if (status.matched_area_count == 0) {
            status.area_name = area.name;
            status.area_type = area.type;
          }
          status.is_in_area = true;
          status.matched_area_count++;
          status.matched_area_names.push_back(area.name);
          status.matched_area_types.push_back(area.type);

          if (print_debug_) {
            RCLCPP_INFO(this->get_logger(),
                        "In area: %s (%s)", area.name.c_str(), area.type.c_str());
          }
        }
      }
    }

    status_pub_->publish(status);

    // 可视化
    if (publish_marker_) {
      visualization_msgs::msg::MarkerArray markers;

      // 机器人位置球体
      if (tf_ok) {
        markers.markers.push_back(
            createRobotPositionMarker(robot_x, robot_y, status.is_in_area,
                                      map_frame_, now));
      }

      // 区域边框 + 文字
      auto area_markers = createAreaMarkers(areas_, status.matched_area_names,
                                            map_frame_, now);
      for (auto& m : area_markers.markers) {
        markers.markers.push_back(std::move(m));
      }

      marker_pub_->publish(markers);
    }
  }

  // 成员
  std::vector<Area> areas_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<rm_decision_interfaces::msg::RobotAreaStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string map_frame_;
  std::string robot_frame_;
  std::string target_area_type_;
  std::string target_area_name_;
  double tf_timeout_;
  bool publish_marker_;
  bool print_debug_;
};

}  // namespace robot_area_detector

RCLCPP_COMPONENTS_REGISTER_NODE(robot_area_detector::RobotAreaDetectorNode)
