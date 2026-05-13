#ifndef POLYGON_MANAGER__MARKER_UTILS_HPP_
#define POLYGON_MANAGER__MARKER_UTILS_HPP_

#include <vector>
#include <string>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "polygon_area.hpp"

namespace polygon_manager {

/**
 * @class MarkerUtils
 * @brief RViz 可视化标记工具类
 * 
 * 负责将多边形区域转换为 RViz Marker，实现地图上的可视化显示
 * 
 * 功能特点：
 * ========
 * 1. 自动闭环：多边形自动闭合（首尾相连）
 * 2. 颜色管理：根据区域类型自动选择颜色
 * 3. ID 管理：每个 Marker 有唯一 ID
 * 4. 坐标系：统一使用 map 坐标系
 * 5. LINE_STRIP：使用线条表示多边形边界
 * 
 * 颜色定义：
 * ========
 * - FORBIDDEN_AREA: 红色 RGB(1.0, 0.0, 0.0)
 * - PROTECT_AREA: 绿色 RGB(0.0, 1.0, 0.0)
 * - PATROL_AREA: 蓝色 RGB(0.0, 0.0, 1.0)
 * - ATTACK_AREA: 黄色 RGB(1.0, 1.0, 0.0)
 * 
 * 使用示例：
 * =======
 * @code
 * PolygonArea area;
 * area.id = 1;
 * area.name = "forbidden_zone";
 * area.type = AreaType::FORBIDDEN_AREA;
 * // ... 添加顶点 ...
 * 
 * visualization_msgs::msg::Marker marker = 
 *     MarkerUtils::createMarkerFromPolygon(area);
 * 
 * publisher->publish(marker);
 * @endcode
 */
class MarkerUtils {
 public:
  /**
   * @brief 从多边形区域创建 RViz Marker
   * 
   * 将 PolygonArea 结构转换为 visualization_msgs::msg::Marker
   * 用于在 RViz 中可视化显示多边形区域
   * 
   * 实现细节：
   * ========
   * 1. Marker 类型：LINE_STRIP
   *    - 连续线条绘制，会自动连接相邻顶点
   *    - 不会自动闭合，所以需要手动添加首顶点作为最后一个点
   * 
   * 2. 坐标系：map
   *    - 统一使用 map 坐标系，支持 SLAM 和离线地图
   *    - 不使用 odom 坐标系（原因：odom 会随机器人移动而变化）
   * 
   * 3. 闭合原理：
   *    为了形成闭合的多边形，需要：
   *    - 添加所有顶点 vertices[0] 到 vertices[n-1]
   *    - 再添加首顶点 vertices[0] 使线条闭合
   * 
   * 示例：
   *    原多边形顶点：(0,0), (1,0), (1,1), (0,1)
   *    Marker 会绘制：(0,0)→(1,0)→(1,1)→(0,1)→(0,0)
   * 
   * 4. 颜色设置：
   *    根据 area.type 自动选择合适的颜色
   * 
   * 5. 其他参数：
   *    - lifetime = 0（持久显示）
   *    - scale.x = 0.02（线宽）
   *    - alpha = 1.0（完全不透明）
   * 
   * @param area 多边形区域数据结构
   * 
   * @return visualization_msgs::msg::Marker 可发布的标记
   * 
   * @note Marker ID 使用 area.id
   * @note 颜色由 getColorByType() 自动确定
   * @note 线宽固定为 0.02 米
   */
  static visualization_msgs::msg::Marker createMarkerFromPolygon(
      const PolygonArea& area);

  /**
   * @brief 从多个多边形区域创建 MarkerArray
   * 
   * 批量将多个 PolygonArea 转换为 MarkerArray
   * 用于在 RViz 中同时显示多个区域
   * 
   * @param areas 多边形区域向量
   * 
   * @return visualization_msgs::msg::MarkerArray 包含所有多边形的标记数组
   * 
   * @note 时间戳使用当前时间
   * @note 每个 Marker 的 ID 保持与对应的 PolygonArea.id 一致
   */
  static visualization_msgs::msg::MarkerArray createMarkerArrayFromPolygons(
      const std::vector<PolygonArea>& areas);

  /**
   * @brief 根据区域类型获取对应的颜色
   * 
   * 颜色方案：
   * --------
   * - FORBIDDEN_AREA(禁区) → 红色 (1.0, 0.0, 0.0, 1.0)
   * - PROTECT_AREA(防守区) → 绿色 (0.0, 1.0, 0.0, 1.0)
   * - PATROL_AREA(巡逻区) → 蓝色 (0.0, 0.0, 1.0, 1.0)
   * - ATTACK_AREA(攻击区) → 黄色 (1.0, 1.0, 0.0, 1.0)
   * 
   * @param type 区域类型
   * 
   * @return std_msgs::msg::ColorRGBA 对应的颜色（包括 alpha 通道）
   * 
   * @note 所有颜色的 alpha 通道都是 1.0（完全不透明）
   * @note 可以通过修改此函数来定制颜色方案
   */
  static std_msgs::msg::ColorRGBA getColorByType(AreaType type);

  /**
   * @brief 创建删除 Marker 的请求
   * 
   * 用于删除 RViz 中的特定多边形显示
   * 
   * @param marker_id 要删除的 Marker ID
   * 
   * @return visualization_msgs::msg::Marker DELETE 操作的标记
   * 
   * @note action = visualization_msgs::msg::Marker::DELETE
   * @note namespace = "polygon_areas"
   */
  static visualization_msgs::msg::Marker createDeleteMarker(
      uint32_t marker_id);

  /**
   * @brief 创建清空所有 Marker 的请求
   * 
   * 删除 RViz 中的所有多边形显示
   * 
   * @return visualization_msgs::msg::Marker DELETE_ALL 操作的标记
   * 
   * @note action = visualization_msgs::msg::Marker::DELETEALL
   */
  static visualization_msgs::msg::Marker createDeleteAllMarkers();

 private:
  /// Marker 命名空间（用于 RViz 识别）
  static constexpr const char* MARKER_NAMESPACE = "polygon_areas";

  /// 线条宽度（米）
  static constexpr float LINE_WIDTH = 0.02f;

  /// 线条宽度缩放因子（用于 scale.x）
  static constexpr float SCALE_X = 0.02f;
};

}  // namespace polygon_manager

#endif  // POLYGON_MANAGER__MARKER_UTILS_HPP_
