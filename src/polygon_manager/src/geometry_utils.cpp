#include <cmath>
#include "polygon_manager/geometry_utils.hpp"

namespace polygon_manager {

/**
 * 射线法（Ray Casting Algorithm）点在多边形内判定的核心实现
 * 
 * 算法流程：
 * ========
 * 1. 从目标点 (x, y) 向右（+x 方向）发射一条射线
 * 2. 遍历多边形的每条边
 * 3. 统计射线与边的交点数
 * 4. 交点数为奇数 → 点在多边形内
 * 5. 交点数为偶数 → 点在多边形外
 * 
 * 为什么这个方法有效？
 * ===================
 * 
 * 根据拓扑学原理：
 * - 如果点在多边形外，任何穿过多边形的射线必然进出相等次数（偶数次）
 * - 如果点在多边形内，射线最后一段只穿进去没穿出来（奇数次）
 * 
 * 举例（1D 类比）：
 * 想象一条线，两端是 "外"，中间是 "内"
 * 从左边界穿进去 1 次，从右边界穿出来 1 次
 * 如果你在中间任取一点向右射线，只穿出 1 次（奇数）
 * 如果你在左边任取一点向右射线，穿出 2 次（偶数，进和出）
 * 
 * 边界情况处理：
 * ============
 * 1. 射线经过顶点：
 *    只有一端的边在射线下方，才计算为交点
 *    避免在顶点处重复计算
 * 
 * 2. 点恰好在边上：
 *    通过检查点到边的距离来判定（使用 EPSILON）
 * 
 * 3. 浮点精度问题：
 *    使用 EPSILON = 1e-10 来处理浮点比较
 */

bool GeometryUtils::isPointInPolygon(
    double x,
    double y,
    const std::vector<geometry_msgs::msg::Point>& polygon) {
  // 参数检查
  if (polygon.size() < 3) {
    return false;
  }

  // 检查点是否在多边形边界上
  for (size_t i = 0; i < polygon.size(); ++i) {
    size_t next = (i + 1) % polygon.size();
    double dist = pointToLineDistance(
        x, y,
        polygon[i].x, polygon[i].y,
        polygon[next].x, polygon[next].y);
    
    if (dist < EPSILON) {
      // 点在边上，认为在多边形内
      return true;
    }
  }

  // 射线法：统计射线与多边形边的交点数
  int intersection_count = 0;

  for (size_t i = 0; i < polygon.size(); ++i) {
    size_t next = (i + 1) % polygon.size();
    
    const geometry_msgs::msg::Point& p1 = polygon[i];
    const geometry_msgs::msg::Point& p2 = polygon[next];

    // 检查边是否与水平射线相交
    // 条件 1：边的 y 坐标范围必须包含点的 y 坐标
    if (!isInYRange(y, p1.y, p2.y)) {
      continue;
    }

    // 条件 2：计算交点 x 坐标
    double intersection_x = calculateIntersectionX(
        x, y,
        p1.x, p1.y,
        p2.x, p2.y);

    // 如果交点在点的右侧（射线方向），计数+1
    if (intersection_x > x + EPSILON) {
      intersection_count++;
    }
  }

  // 交点数为奇数，点在多边形内
  return (intersection_count % 2) == 1;
}

/**
 * 计算点到线段的垂直距离
 * 
 * 使用点到直线的距离公式：
 * 点 P(px, py) 到直线 Ax + By + C = 0 的距离为：
 * d = |Apx + Bpy + C| / sqrt(A^2 + B^2)
 * 
 * 对于通过两点 (x1,y1) 和 (x2,y2) 的直线：
 * A = y2 - y1
 * B = x1 - x2
 * C = x2*y1 - x1*y2
 */
double GeometryUtils::pointToLineDistance(
    double px, double py,
    double x1, double y1,
    double x2, double y2) {
  double dx = x2 - x1;
  double dy = y2 - y1;

  // 线段长度的平方
  double length_sq = dx * dx + dy * dy;

  if (length_sq < EPSILON) {
    // 线段退化为点，返回点到该点的距离
    double dpx = px - x1;
    double dpy = py - y1;
    return std::sqrt(dpx * dpx + dpy * dpy);
  }

  // 使用向量叉积计算距离
  // |向量 P1P × 向量 P1P2| / |向量 P1P2|
  double cross_product = std::abs(
      (y1 - py) * dx - (x1 - px) * dy);
  
  double line_length = std::sqrt(length_sq);
  
  return cross_product / line_length;
}

/**
 * 获取浮点精度常量
 * 用于判断两个浮点数是否相等
 */
double GeometryUtils::getEpsilon() {
  return EPSILON;
}

/**
 * 检查两个浮点数是否相等（在精度范围内）
 */
bool GeometryUtils::isEqual(double a, double b) {
  return std::abs(a - b) < EPSILON;
}

/**
 * 验证多边形是否有效
 * 
 * 检查项：
 * 1. 顶点数 >= 3
 * 2. 相邻顶点不重合
 * 3. 首尾顶点不相同
 */
bool GeometryUtils::isValidPolygon(
    const std::vector<geometry_msgs::msg::Point>& polygon) {
  if (polygon.size() < 3) {
    return false;
  }

  // 检查相邻顶点是否重合
  for (size_t i = 0; i < polygon.size(); ++i) {
    size_t next = (i + 1) % polygon.size();
    const auto& p1 = polygon[i];
    const auto& p2 = polygon[next];
    
    if (isEqual(p1.x, p2.x) && isEqual(p1.y, p2.y)) {
      return false;  // 相邻顶点重合
    }
  }

  return true;
}

/**
 * 检查点是否在线段的垂直范围内
 * 
 * 用于射线法中的前置检查：
 * 只有当点的 y 坐标在线段 y 范围内时，才可能有交点
 * 
 * 处理边界情况：不包括较大的 y 值端点
 * 这样可以避免在顶点处重复计算
 */
bool GeometryUtils::isInYRange(double py, double y1, double y2) {
  // 使射线的 y 坐标在两个端点的范围内
  // 但只包括一端（小于号），避免顶点重复
  if (y1 > y2) {
    return py >= y2 && py < y1;
  } else {
    return py >= y1 && py < y2;
  }
}

/**
 * 计算线段与水平射线的交点 x 坐标
 * 
 * 射线：从 (px, py) 向右 +x 方向
 * 线段：从 (x1, y1) 到 (x2, y2)
 * 
 * 使用参数方程求交点：
 * 线段参数方程：
 *   X(t) = x1 + t(x2 - x1)
 *   Y(t) = y1 + t(y2 - y1)  其中 0 <= t <= 1
 * 
 * 射线方程：
 *   Y = py
 * 
 * 求解 Y(t) = py：
 *   y1 + t(y2 - y1) = py
 *   t = (py - y1) / (y2 - y1)
 * 
 * 代入 X(t) 得交点 x 坐标：
 *   x = x1 + t(x2 - x1)
 *     = x1 + ((py - y1) / (y2 - y1))(x2 - x1)
 */
double GeometryUtils::calculateIntersectionX(
    double px, double py,
    double x1, double y1,
    double x2, double y2) {
  // 参数 t
  double t = (py - y1) / (y2 - y1);

  // 交点 x 坐标
  double intersection_x = x1 + t * (x2 - x1);

  return intersection_x;
}

}  // namespace polygon_manager
