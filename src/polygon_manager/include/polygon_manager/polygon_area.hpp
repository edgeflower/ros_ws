#ifndef POLYGON_MANAGER__POLYGON_AREA_HPP_
#define POLYGON_MANAGER__POLYGON_AREA_HPP_

#include <vector>
#include <string>
#include <geometry_msgs/msg/point.hpp>

namespace polygon_manager {

/**
 * @enum AreaType
 * @brief 区域类型枚举
 * 
 * 定义了四种区域类型：
 * - FORBIDDEN_AREA: 禁区（敌方进入触发警报）
 * - PROTECT_AREA: 防守区（需要重点保护的区域）
 * - PATROL_AREA: 巡逻区（定期巡逻的区域）
 * - ATTACK_AREA: 攻击区（敌方可以进入的区域）
 */
enum class AreaType {
  FORBIDDEN_AREA = 0,  ///< 禁区 - 敌方绝对不能进入
  PROTECT_AREA = 1,    ///< 防守区 - 需要保护的区域
  PATROL_AREA = 2,     ///< 巡逻区 - 巡逻路径
  ATTACK_AREA = 3      ///< 攻击区 - 可以主动进攻的区域
};

/**
 * @struct PolygonArea
 * @brief 多边形区域数据结构
 * 
 * 用于表示地图上的一个多边形区域。
 * 包含：
 * - 区域的唯一标识符
 * - 区域类型（禁区/防守区/巡逻区/攻击区）
 * - 多边形顶点列表（按顺序存储）
 * - 区域名称
 * 
 * 使用说明：
 * 1. 创建新区域时，按顺序添加多边形顶点
 * 2. vertices 向量存储的顺序很重要（逆时针或顺时针要一致）
 * 3. 支持凸多边形和凹多边形
 */
struct PolygonArea {
  uint32_t id;                                           ///< 唯一标识符
  std::string name;                                      ///< 区域名称（例如："forbidden_zone_1"）
  AreaType type;                                         ///< 区域类型
  std::vector<geometry_msgs::msg::Point> vertices;      ///< 多边形顶点列表

  /**
   * @brief 获取区域类型的字符串表示
   * @return 返回区域类型的字符串（如 "FORBIDDEN_AREA"）
   */
  std::string getTypeString() const {
    switch (type) {
      case AreaType::FORBIDDEN_AREA:
        return "FORBIDDEN_AREA";
      case AreaType::PROTECT_AREA:
        return "PROTECT_AREA";
      case AreaType::PATROL_AREA:
        return "PATROL_AREA";
      case AreaType::ATTACK_AREA:
        return "ATTACK_AREA";
      default:
        return "UNKNOWN";
    }
  }

  /**
   * @brief 检查多边形是否有效
   * @return true 如果多边形至少有 3 个顶点，false 否则
   * 
   * 一个有效的多边形必须至少有 3 个顶点（三角形）
   */
  bool isValid() const {
    return vertices.size() >= 3;
  }

  /**
   * @brief 清空多边形的所有顶点
   */
  void clear() {
    vertices.clear();
  }

  /**
   * @brief 获取顶点数量
   * @return 多边形的顶点数
   */
  size_t getVertexCount() const {
    return vertices.size();
  }
};

}  // namespace polygon_manager

#endif  // POLYGON_MANAGER__POLYGON_AREA_HPP_
