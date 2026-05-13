#ifndef ENEMY_FORBIDDEN_AREA_DETECTOR__MARKER_UTILS_HPP_
#define ENEMY_FORBIDDEN_AREA_DETECTOR__MARKER_UTILS_HPP_

#include <vector>
#include <string>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/time.hpp>

namespace enemy_forbidden_area_detector {

/**
 * @struct ForbiddenPolygon
 * @brief 存储从 YAML 加载的单个 forbidden 多边形
 */
struct ForbiddenPolygon {
  std::string name;
  std::vector<geometry_msgs::msg::Point> vertices;
};

/**
 * @class MarkerUtils
 * @brief RViz 可视化工具类
 *
 * 只负责发布检测相关的可视化，不包含编辑功能：
 * 1. forbidden polygon 边界线（红色 LINE_STRIP）
 * 2. 敌方位置球体（红/绿 SPHERE）
 * 3. 状态文字（TEXT_VIEW_FACING）
 *
 * Marker 基础知识：
 * - frame_id: Marker 的参考坐标系，设为 "map" 表示在地图坐标系下显示
 * - ns (namespace): Marker 的命名空间，用于区分不同组的 Marker
 * - id: 同一 ns 内的唯一标识，用于更新或删除特定 Marker
 * - action: ADD=添加/更新，DELETE=删除单个，DELETEALL=删除全部
 */
class MarkerUtils {
 public:
  /**
   * @brief 创建所有 forbidden polygon 的 LINE_STRIP MarkerArray
   *
   * LINE_STRIP 会按顺序连接所有顶点画线。
   * 自动闭环：在末尾再添加一次第一个顶点，使首尾相连。
   */
  static visualization_msgs::msg::MarkerArray createForbiddenAreaMarkers(
      const std::vector<ForbiddenPolygon>& polygons,
      const rclcpp::Time& stamp);

  /**
   * @brief 创建敌方位置球体 Marker
   *
   * 在禁区内显示红色，在禁区外显示绿色
   * 无目标时返回 DELETE marker（从 RViz 中移除）
   */
  static visualization_msgs::msg::Marker createEnemyPositionMarker(
      double x, double y,
      bool is_forbidden,
      bool has_target,
      const rclcpp::Time& stamp);

  /**
   * @brief 创建状态文字 Marker
   *
   * 跟随敌方位置显示 "FORBIDDEN" 或 "SAFE"
   * 无目标时返回 DELETE marker
   */
  static visualization_msgs::msg::Marker createStatusTextMarker(
      double x, double y,
      bool is_forbidden,
      bool has_target,
      const rclcpp::Time& stamp);

  /**
   * @brief 创建 DELETEALL marker（用于清理所有可视化）
   */
  static visualization_msgs::msg::Marker createDeleteAllMarker();
};

}  // namespace enemy_forbidden_area_detector

#endif  // ENEMY_FORBIDDEN_AREA_DETECTOR__MARKER_UTILS_HPP_
