#ifndef ENEMY_FORBIDDEN_AREA_DETECTOR__GEOMETRY_UTILS_HPP_
#define ENEMY_FORBIDDEN_AREA_DETECTOR__GEOMETRY_UTILS_HPP_

#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace enemy_forbidden_area_detector {

/**
 * @class GeometryUtils
 * @brief 几何工具类，提供点在多边形内判定算法
 *
 * 核心算法：射线法（Ray Casting Algorithm）
 *
 * 原理：
 * 从目标点向右（x 正方向）发射一条水平射线，
 * 统计射线与多边形边界的交点数量：
 *   - 交点数为奇数 → 点在多边形内部
 *   - 交点数为偶数 → 点在多边形外部
 *
 * 数学推导：
 * 对于多边形的每条边 (x1,y1)-(x2,y2)，检查：
 * 1. 目标点的 y 坐标是否在这条边的 y 范围内
 * 2. 如果在，计算射线与该边的交点 x 坐标
 * 3. 如果交点在目标点右侧，则计为一个交点
 *
 * 时间复杂度：O(n)，n 为多边形顶点数
 * 支持凸多边形和凹多边形
 * 使用 EPSILON = 1e-10 处理浮点误差
 */
class GeometryUtils {
 public:
  /**
   * @brief 使用射线法判断点是否在多边形内部
   *
   * @param x 目标点的 x 坐标（map 坐标系）
   * @param y 目标点的 y 坐标（map 坐标系）
   * @param polygon 多边形顶点列表（至少 3 个顶点）
   *
   * @return true 点在多边形内部，false 点在多边形外部
   */
  static bool isPointInPolygon(
      double x, double y,
      const std::vector<geometry_msgs::msg::Point>& polygon);

  /**
   * @brief 验证多边形是否有效（至少 3 个顶点）
   */
  static bool isValidPolygon(
      const std::vector<geometry_msgs::msg::Point>& polygon);

 private:
  static constexpr double EPSILON = 1e-10;
};

}  // namespace enemy_forbidden_area_detector

#endif  // ENEMY_FORBIDDEN_AREA_DETECTOR__GEOMETRY_UTILS_HPP_
