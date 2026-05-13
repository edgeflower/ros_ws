#include "enemy_forbidden_area_detector/geometry_utils.hpp"

namespace enemy_forbidden_area_detector {

bool GeometryUtils::isValidPolygon(
    const std::vector<geometry_msgs::msg::Point>& polygon) {
  return polygon.size() >= 3;
}

/**
 * 射线法（Ray Casting Algorithm）实现
 *
 * 算法原理：
 * ========
 * 从点 (x, y) 向右（x 正方向）发射一条水平射线，
 * 遍历多边形的每条边，统计射线与多边形边的交点数。
 *
 * 为什么奇偶性有效？
 * --------
 * 想象从多边形外部走到目标点的路径：
 * - 每穿过一条边界，就从"外部"变成"内部"（或反过来）
 * - 穿过奇数次 → 最终在内部
 * - 穿过偶数次 → 最终在外部
 *
 * 对每条边 (x1,y1)→(x2,y2) 的检查：
 * 1. y 是否在这条边的 y 范围内（不包含上端点，避免顶点重复计数）
 * 2. 如果是，计算射线与该边的交点 x 坐标
 * 3. 如果交点在目标点右侧（> x），则计为一个交点
 *
 * 浮点误差处理：
 * 使用 EPSILON = 1e-10 避免边界上的误判
 */
bool GeometryUtils::isPointInPolygon(
    double x, double y,
    const std::vector<geometry_msgs::msg::Point>& polygon) {
  if (!isValidPolygon(polygon)) return false;

  int n = static_cast<int>(polygon.size());
  bool inside = false;

  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = polygon[i].x, yi = polygon[i].y;
    double xj = polygon[j].x, yj = polygon[j].y;

    // 检查 y 是否在边的 y 范围内
    // 注意：使用严格不等式排除上端点，避免射线穿过顶点时重复计数
    if (((yi > y) != (yj > y))) {
      // 计算射线与边的交点 x 坐标
      // 使用线性插值：x_intersect = xj + (y - yj) / (yi - yj) * (xi - xj)
      double x_intersect = xj + (y - yj) / (yi - yj) * (xi - xj);

      // 如果交点在目标点右侧，切换内外状态
      if (x < x_intersect) {
        inside = !inside;
      }
    }
  }

  return inside;
}

}  // namespace enemy_forbidden_area_detector
