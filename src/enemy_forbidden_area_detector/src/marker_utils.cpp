#include "enemy_forbidden_area_detector/marker_utils.hpp"

namespace enemy_forbidden_area_detector {

visualization_msgs::msg::MarkerArray MarkerUtils::createForbiddenAreaMarkers(
    const std::vector<ForbiddenPolygon>& polygons,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < polygons.size(); ++i) {
    const auto& polygon = polygons[i];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = stamp;
    marker.ns = "forbidden_area";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 红色线条
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;

    // 线宽
    marker.scale.x = 0.05;

    // 添加所有顶点
    for (const auto& vertex : polygon.vertices) {
      geometry_msgs::msg::Point p;
      p.x = vertex.x;
      p.y = vertex.y;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    // 自动闭环：把第一个顶点再加到末尾，使 LINE_STRIP 首尾相连
    if (!polygon.vertices.empty()) {
      geometry_msgs::msg::Point p;
      p.x = polygon.vertices[0].x;
      p.y = polygon.vertices[0].y;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::Marker MarkerUtils::createEnemyPositionMarker(
    double x, double y,
    bool is_forbidden,
    bool has_target,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = "enemy_position";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;

  // 无目标时删除 marker
  if (!has_target) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  marker.action = visualization_msgs::msg::Marker::ADD;

  // 敌人在禁区 → 红色，在禁区外 → 绿色
  marker.color.r = is_forbidden ? 1.0f : 0.0f;
  marker.color.g = is_forbidden ? 0.0f : 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  // 球体大小
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // 位置
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;

  return marker;
}

visualization_msgs::msg::Marker MarkerUtils::createStatusTextMarker(
    double x, double y,
    bool is_forbidden,
    bool has_target,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = "enemy_status_text";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

  if (!has_target) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.text = is_forbidden ? "FORBIDDEN" : "SAFE";

  // 文字颜色与球体一致
  marker.color.r = is_forbidden ? 1.0f : 0.0f;
  marker.color.g = is_forbidden ? 0.0f : 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  // 文字大小
  marker.scale.z = 0.5;

  // 文字位置（在敌人位置上方偏移一点）
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.5;
  marker.pose.orientation.w = 1.0;

  return marker;
}

visualization_msgs::msg::Marker MarkerUtils::createDeleteAllMarker() {
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace enemy_forbidden_area_detector
