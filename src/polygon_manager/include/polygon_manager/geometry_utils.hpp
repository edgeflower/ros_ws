#ifndef POLYGON_MANAGER__GEOMETRY_UTILS_HPP_
#define POLYGON_MANAGER__GEOMETRY_UTILS_HPP_

#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include "polygon_area.hpp"

namespace polygon_manager {

/**
 * @class GeometryUtils
 * @brief 几何算法工具类
 * 
 * 提供核心的几何计算功能：
 * 1. 点在多边形内的判定（射线法）
 * 2. 线段和点的距离计算
 * 3. 浮点数精度处理
 * 
 * 所有算法均为O(n)时间复杂度，支持凸多边形和凹多边形
 * 
 * 使用示例：
 * @code
 * geometry_msgs::msg::Point p;
 * p.x = 2.5; p.y = 2.5;
 * std::vector<geometry_msgs::msg::Point> polygon = {...};
 * 
 * bool inside = GeometryUtils::isPointInPolygon(p.x, p.y, polygon);
 * @endcode
 */
class GeometryUtils {
 public:
  /**
   * @brief 使用射线法判断点是否在多边形内
   * 
   * 算法原理（射线法 - Ray Casting Algorithm）：
   * ================================================
   * 
   * 从目标点出发，向任意方向（通常向右/x正方向）发射一条射线。
   * 统计这条射线与多边形边界的交点数：
   * 
   * - 交点数为奇数 → 点在多边形内
   * - 交点数为偶数 → 点在多边形外
   * 
   * 原理解释：
   * --------
   * 如果点在多边形外，射线必然交多边形边界偶数次（进出相等）
   * 如果点在多边形内，射线必然交多边形边界奇数次（最后一次只进不出）
   * 
   * 数学推导：
   * --------
   * 考虑线段 (y1, y2) 和射线起点 py：
   * 1. 线段的 y 坐标范围应包含 py
   * 2. 线段与射线的交点 x 坐标应在 px 右侧
   * 
   * 边界情况处理：
   * ---------
   * - 当射线经过多边形顶点时需要特殊处理
   * - 当点恰好在边上时需要处理（浮点误差）
   * 
   * @param x 点的 x 坐标
   * @param y 点的 y 坐标
   * @param polygon 多边形顶点向量
   * 
   * @return true 点在多边形内，false 点在多边形外
   * 
   * @note 时间复杂度：O(n)，其中 n 是多边形顶点数
   * @note 空间复杂度：O(1)
   * @note 支持凸多边形和凹多边形
   * @note 浮点误差处理：使用 EPSILON = 1e-10 用于边界判断
   */
  static bool isPointInPolygon(
      double x,
      double y,
      const std::vector<geometry_msgs::msg::Point>& polygon);

  /**
   * @brief 计算点到线段的垂直距离
   * 
   * 用于边界情况判定，当点恰好在多边形边上时使用
   * 
   * @param px 点的 x 坐标
   * @param py 点的 y 坐标
   * @param x1 线段起点 x 坐标
   * @param y1 线段起点 y 坐标
   * @param x2 线段终点 x 坐标
   * @param y2 线段终点 y 坐标
   * 
   * @return 点到线段的距离（浮点数）
   */
  static double pointToLineDistance(
      double px, double py,
      double x1, double y1,
      double x2, double y2);

  /**
   * @brief 获取当前使用的浮点精度值
   * 
   * 用于判断浮点数是否接近零（考虑浮点误差）
   * 当 |a - b| < EPSILON 时，认为 a ≈ b
   * 
   * @return 返回当前的 EPSILON 值（默认 1e-10）
   */
  static double getEpsilon();

  /**
   * @brief 检查两个浮点数是否相等（考虑误差）
   * 
   * @param a 第一个数
   * @param b 第二个数
   * @return true 如果 |a - b| < EPSILON
   */
  static bool isEqual(double a, double b);

  /**
   * @brief 验证多边形是否有效
   * 
   * 检查条件：
   * 1. 至少有 3 个顶点
   * 2. 相邻顶点不重合
   * 3. 首尾顶点不相同
   * 
   * @param polygon 多边形顶点向量
   * @return true 如果多边形有效
   */
  static bool isValidPolygon(
      const std::vector<geometry_msgs::msg::Point>& polygon);

 private:
  /// 浮点数精度常量（用于处理浮点运算误差）
  static constexpr double EPSILON = 1e-10;

  /**
   * @brief 检查点是否在线段的垂直范围内（y 方向）
   * 
   * 辅助函数，用于射线法中的线段判定
   * 
   * @param py 点的 y 坐标
   * @param y1 线段起点的 y 坐标
   * @param y2 线段终点的 y 坐标
   * 
   * @return true 如果 py 在 y1 和 y2 的范围内（不包括较大值的那一端）
   */
  static bool isInYRange(double py, double y1, double y2);

  /**
   * @brief 计算线段与水平射线的交点 x 坐标
   * 
   * 辅助函数，用于射线法中的交点计算
   * 给定从点 (px, py) 出发向右的射线，计算它与线段的交点
   * 
   * @param px 射线起点的 x 坐标
   * @param py 射线起点的 y 坐标
   * @param x1 线段起点的 x 坐标
   * @param y1 线段起点的 y 坐标
   * @param x2 线段终点的 x 坐标
   * @param y2 线段终点的 y 坐标
   * 
   * @return 交点的 x 坐标（如果射线不与线段相交则返回 -1）
   */
  static double calculateIntersectionX(
      double px, double py,
      double x1, double y1,
      double x2, double y2);
};

}  // namespace polygon_manager

#endif  // POLYGON_MANAGER__GEOMETRY_UTILS_HPP_
