#include "polygon_manager/marker_utils.hpp"
#include <std_msgs/msg/color_rgba.hpp>
#include <rclcpp/clock.hpp>

namespace polygon_manager {

/**
 * 从多边形创建 RViz Marker
 * 
 * 关键实现细节：
 * ============
 * 
 * 1. LINE_STRIP 类型：
 *    visualization_msgs::msg::Marker::LINE_STRIP
 *    - 按顺序连接所有顶点
 *    - 自动在相邻顶点之间绘制线条
 *    - 但 LINE_STRIP 本身不会自动闭合
 *    - 所以需要手动在 points 数组末尾添加首顶点
 * 
 * 2. 闭合多边形的方法：
 *    原多边形顶点：v0, v1, v2, ..., vn
 *    Marker points：v0, v1, v2, ..., vn, v0
 *    这样 LINE_STRIP 会自动绘制闭合的多边形
 * 
 * 3. 坐标系统一：
 *    frame_id = "map"
 *    - 使用全局坐标系
 *    - 不使用 "odom"，因为 odom 会随机器人移动而变化
 *    - 地图上的禁区是绝对位置，需要绝对坐标系
 * 
 * 4. Marker 参数：
 *    - id：使用 area.id，保证唯一性
 *    - namespace："polygon_areas"，方便 RViz 分组
 *    - action：ADD/MODIFY，自动处理更新
 *    - lifetime：Duration(0) 表示持久显示
 *    - scale.x：线宽，设为 0.02 米
 *    - color：根据区域类型自动选择
 */

visualization_msgs::msg::Marker MarkerUtils::createMarkerFromPolygon(
    const PolygonArea& area) {
  visualization_msgs::msg::Marker marker;

  // 设置基本属性
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  
  marker.ns = MARKER_NAMESPACE;
  marker.id = area.id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 持久显示（lifetime = 0）
  marker.lifetime = rclcpp::Duration(0, 0);

  // 设置线宽
  marker.scale.x = SCALE_X;  // 线宽
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  // 设置颜色
  marker.color = getColorByType(area.type);

  // 添加所有顶点（实现闭合）
  for (const auto& vertex : area.vertices) {
    geometry_msgs::msg::Point p;
    p.x = vertex.x;
    p.y = vertex.y;
    p.z = 0.0;  // 2D 地图，z=0
    marker.points.push_back(p);
  }

  // 添加首顶点以实现闭合
  // 这样 LINE_STRIP 会形成闭合的多边形
  if (!area.vertices.empty()) {
    geometry_msgs::msg::Point closing_point;
    closing_point.x = area.vertices[0].x;
    closing_point.y = area.vertices[0].y;
    closing_point.z = 0.0;
    marker.points.push_back(closing_point);
  }

  return marker;
}

/**
 * 从多个多边形创建 MarkerArray
 * 
 * 用于批量可视化所有定义的区域
 */
visualization_msgs::msg::MarkerArray MarkerUtils::createMarkerArrayFromPolygons(
    const std::vector<PolygonArea>& areas) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (const auto& area : areas) {
    marker_array.markers.push_back(createMarkerFromPolygon(area));
  }

  return marker_array;
}

/**
 * 根据区域类型获取颜色
 * 
 * 颜色方案（标准机器人竞赛配色）：
 * ============================
 * 禁区 (FORBIDDEN_AREA)：红色 RGB(1.0, 0.0, 0.0)
 *   - 表示危险区域，需要避开
 *   - 红色是警告色，用户能快速识别
 * 
 * 防守区 (PROTECT_AREA)：绿色 RGB(0.0, 1.0, 0.0)
 *   - 表示需要保护的己方阵地
 *   - 绿色表示安全和防守
 * 
 * 巡逻区 (PATROL_AREA)：蓝色 RGB(0.0, 0.0, 1.0)
 *   - 表示可以活动的区域
 *   - 蓝色表示中立和自由
 * 
 * 攻击区 (ATTACK_AREA)：黄色 RGB(1.0, 1.0, 0.0)
 *   - 表示可以主动攻击的区域
 *   - 黄色表示警惕和行动
 * 
 * alpha：所有颜色都设为 1.0（完全不透明）
 *       可根据需要修改为 0.5 以显示半透明效果
 */
std_msgs::msg::ColorRGBA MarkerUtils::getColorByType(AreaType type) {
  std_msgs::msg::ColorRGBA color;
  color.a = 1.0;  // 完全不透明

  switch (type) {
    case AreaType::FORBIDDEN_AREA:
      // 禁区：红色
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      break;

    case AreaType::PROTECT_AREA:
      // 防守区：绿色
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
      break;

    case AreaType::PATROL_AREA:
      // 巡逻区：蓝色
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      break;

    case AreaType::ATTACK_AREA:
      // 攻击区：黄色
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.0;
      break;

    default:
      // 未知类型：灰色
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.5;
  }

  return color;
}

/**
 * 创建删除特定 Marker 的请求
 * 
 * 通过设置 action = MARKER::DELETE 来指示 RViz
 * 删除指定 ID 的 Marker
 */
visualization_msgs::msg::Marker MarkerUtils::createDeleteMarker(
    uint32_t marker_id) {
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  
  marker.ns = MARKER_NAMESPACE;
  marker.id = marker_id;
  marker.action = visualization_msgs::msg::Marker::DELETE;

  return marker;
}

/**
 * 创建删除所有 Marker 的请求
 * 
 * 通过设置 action = Marker::DELETEALL 来指示 RViz
 * 删除所有该 namespace 下的 Marker
 */
visualization_msgs::msg::Marker MarkerUtils::createDeleteAllMarkers() {
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Clock().now();
  
  marker.ns = MARKER_NAMESPACE;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;

  return marker;
}

}  // namespace polygon_manager
