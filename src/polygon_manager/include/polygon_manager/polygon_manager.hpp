#ifndef POLYGON_MANAGER__POLYGON_MANAGER_HPP_
#define POLYGON_MANAGER__POLYGON_MANAGER_HPP_

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/string.hpp>

#include "polygon_area.hpp"
#include "geometry_utils.hpp"
#include "marker_utils.hpp"

namespace polygon_manager {

/**
 * @class PolygonManager
 * @brief 禁区管理系统的核心类
 * 
 * 功能概述：
 * ========
 * 这是整个禁区管理系统的主要管理类，负责：
 * 
 * 1. 多边形管理：
 *    - 添加/删除多边形区域
 *    - 清空所有区域
 *    - 闭合多边形
 *    - 管理多个区域的集合
 * 
 * 2. 敌方检测：
 *    - 接收敌方位置信息
 *    - 检测敌方是否在禁区内
 *    - 检测敌方所在的区域类型
 * 
 * 3. 配置管理：
 *    - 从 YAML 文件加载配置
 *    - 保存当前配置到 YAML 文件
 *    - 动态编辑区域
 * 
 * 4. 可视化：
 *    - 生成 RViz Marker
 *    - 发布可视化信息
 * 
 * 5. 坐标系管理：
 *    - 支持 slam_toolbox 在线地图（使用 /map topic）
 *    - 支持 nav2_map_server 离线地图（使用 map.yaml）
 *    - 统一使用 map 坐标系（不使用 odom）
 * 
 * 架构说明：
 * ========
 * PolygonManager 是一个纯逻辑类，不包含 ROS 依赖。
 * ROS 节点部分在 PolygonManagerNode 中实现。
 * 这样设计的好处：
 * - 逻辑与 ROS 通信分离
 * - 便于单元测试
 * - 可复用于非 ROS 项目
 * 
 * 坐标系统一说明：
 * ============
 * 为什么统一使用 map 而不是 odom：
 * 
 * odom 坐标系：
 * - 以机器人初始位置为原点
 * - 随着机器人移动而不断变化
 * - 是相对坐标系统
 * - 用于短期定位
 * 
 * map 坐标系：
 * - 以地图的固定位置为原点
 * - 始终保持不变
 * - 是绝对坐标系统
 * - 用于全局导航和区域定义
 * 
 * 在我们的应用中：
 * - 禁区是绝对的地图约束，不应随机器人移动而改变
 * - 必须使用 map 坐标系
 * - 无论是 SLAM 建图还是离线地图，都是在 map 坐标系下定义的
 */
class PolygonManager {
 public:
  /**
   * @brief 构造函数
   * 
   * 初始化一个空的多边形管理器
   */
  PolygonManager();

  /**
   * @brief 析构函数
   */
  ~PolygonManager() = default;

  // ==================== 多边形管理接口 ====================

  /**
   * @brief 添加新的多边形区域
   * 
   * 创建一个新的多边形区域并加入管理
   * 
   * @param name 区域名称（例如："forbidden_zone_1"）
   * @param type 区域类型
   * @param vertices 多边形顶点向量
   * 
   * @return true 添加成功，false 如果多边形无效或名称重复
   * 
   * @note 多边形必须至少有 3 个顶点
   * @note 名称必须唯一（不能重复）
   */
  bool addPolygon(
      const std::string& name,
      AreaType type,
      const std::vector<geometry_msgs::msg::Point>& vertices);

  /**
   * @brief 删除指定的多边形区域
   * 
   * @param polygon_id 要删除的多边形 ID
   * 
   * @return true 删除成功，false 如果 ID 不存在
   */
  bool deletePolygon(uint32_t polygon_id);

  /**
   * @brief 按名称删除多边形区域
   * 
   * @param name 要删除的多边形名称
   * 
   * @return true 删除成功，false 如果名称不存在
   */
  bool deletePolygonByName(const std::string& name);

  /**
   * @brief 清空所有多边形区域
   * 
   * 删除当前管理的所有多边形
   */
  void clearAllPolygons();

  /**
   * @brief 使用临时顶点数据开始创建新的多边形
   * 
   * 调用此函数开始交互式创建多边形
   * 通过多次调用 addVertexToCurrentPolygon() 添加顶点
   */
  void startCreatingPolygon(const std::string& name, AreaType type);

  /**
   * @brief 向当前创建的多边形添加顶点
   * 
   * 每次点击 RViz 中的点时调用
   * 
   * @param point 要添加的顶点
   * 
   * @return 当前多边形已有的顶点数
   */
  size_t addVertexToCurrentPolygon(const geometry_msgs::msg::Point& point);

  /**
   * @brief 闭合当前创建的多边形
   * 
   * 完成多边形创建，将其添加到管理列表
   * 
   * @return true 闭合成功，false 如果顶点不足 3 个
   */
  bool closeCurrentPolygon();

  /**
   * @brief 取消当前多边形的创建
   * 
   * 丢弃临时顶点数据
   */
  void cancelCurrentPolygon();

  // ==================== 敌方检测接口 ====================

  /**
   * @brief 检测敌方是否在禁区内
   * 
   * @param enemy_pose 敌方位置（PoseStamped）
   * 
   * @return true 敌方在禁区内，false 否则
   */
  bool isEnemyInForbiddenArea(const geometry_msgs::msg::PoseStamped& enemy_pose) const;

  /**
   * @brief 检测敌方所在的所有区域类型
   * 
   * @param enemy_pose 敌方位置（PoseStamped）
   * 
   * @return 包含所有包含敌方的区域类型的向量
   * 
   * @note 一个敌方可能同时在多个区域内
   */
  std::vector<AreaType> getEnemyAreas(
      const geometry_msgs::msg::PoseStamped& enemy_pose) const;

  /**
   * @brief 检测敌方所在的所有区域名称
   * 
   * @param enemy_pose 敌方位置
   * 
   * @return 包含所有包含敌方的区域名称的向量
   */
  std::vector<std::string> getEnemyAreaNames(
      const geometry_msgs::msg::PoseStamped& enemy_pose) const;

  /**
   * @brief 生成敌方状态字符串
   * 
   * 用于发布到 /enemy_area_state topic
   * 
   * @param enemy_pose 敌方位置
   * 
   * @return 格式化的状态字符串，例如 "enemy_in_forbidden_area"
   */
  std::string generateEnemyStateString(
      const geometry_msgs::msg::PoseStamped& enemy_pose) const;

  // ==================== 数据查询接口 ====================

  /**
   * @brief 获取所有多边形区域
   * 
   * @return 所有 PolygonArea 的常引用向量
   */
  const std::vector<PolygonArea>& getAllPolygons() const {
    return polygons_;
  }

  /**
   * @brief 按 ID 查找多边形
   * 
   * @param polygon_id 多边形 ID
   * 
   * @return 指向 PolygonArea 的指针，如果不存在返回 nullptr
   */
  const PolygonArea* getPolygonById(uint32_t polygon_id) const;

  /**
   * @brief 按名称查找多边形
   * 
   * @param name 多边形名称
   * 
   * @return 指向 PolygonArea 的指针，如果不存在返回 nullptr
   */
  const PolygonArea* getPolygonByName(const std::string& name) const;

  /**
   * @brief 获取多边形总数
   * 
   * @return 当前管理的多边形数量
   */
  size_t getPolygonCount() const {
    return polygons_.size();
  }

  /**
   * @brief 获取当前创建的多边形的顶点数
   * 
   * 用于交互式编辑时显示当前进度
   * 
   * @return 当前多边形顶点数
   */
  size_t getCurrentPolygonVertexCount() const {
    return current_polygon_vertices_.size();
  }

  /**
   * @brief 获取当前创建的多边形顶点列表
   * 
   * @return 当前多边形顶点的常引用向量
   */
  const std::vector<geometry_msgs::msg::Point>& getCurrentPolygonVertices() const {
    return current_polygon_vertices_;
  }

  // ==================== 可视化接口 ====================

  /**
   * @brief 生成所有多边形的 MarkerArray
   * 
   * 将所有已定义的多边形转换为 RViz 可显示的 Marker
   * 
   * @return visualization_msgs::msg::MarkerArray 用于发布
   */
  visualization_msgs::msg::MarkerArray getMarkerArray() const;

  /**
   * @brief 生成当前创建中的多边形的 Marker
   * 
   * 用于实时显示用户正在编辑的多边形
   * 
   * @return visualization_msgs::msg::Marker 用于发布
   */
  visualization_msgs::msg::Marker getCurrentPolygonMarker() const;

  // ==================== 配置管理接口 ====================

  /**
   * @brief 从 YAML 文件加载多边形配置
   * 
   * 配置文件格式：
   * @code
   * areas:
   *   - name: forbidden_zone_1
   *     type: forbidden
   *     points:
   *       - [1.0, 1.0]
   *       - [5.0, 1.0]
   *       - [5.0, 4.0]
   *       - [1.0, 4.0]
   * 
   *   - name: protect_zone_1
   *     type: protect
   *     points:
   *       - [7.0, 2.0]
   *       - [9.0, 2.0]
   *       - [9.0, 5.0]
   *       - [7.0, 5.0]
   * @endcode
   * 
   * 类型映射：
   * - "forbidden" → FORBIDDEN_AREA
   * - "protect" → PROTECT_AREA
   * - "patrol" → PATROL_AREA
   * - "attack" → ATTACK_AREA
   * 
   * @param yaml_file_path 配置文件路径
   * 
   * @return true 加载成功，false 加载失败或文件不存在
   * 
   * @note 加载前会清空所有现有多边形
   */
  bool loadFromYaml(const std::string& yaml_file_path);

  /**
   * @brief 将当前配置保存到 YAML 文件
   * 
   * @param yaml_file_path 保存路径
   * 
   * @return true 保存成功，false 保存失败
   */
  bool saveToYaml(const std::string& yaml_file_path) const;

  // ==================== 类型转换工具 ====================

  static AreaType stringToAreaType(const std::string& type_str);
  static std::string areaTypeToString(AreaType type);

 private:
  // ==================== 私有成员变量 ====================

  /// 所有多边形区域的列表
  std::vector<PolygonArea> polygons_;

  /// 下一个多边形的 ID（自增）
  uint32_t next_polygon_id_ = 1;

  /// 当前正在创建的多边形的顶点（临时存储）
  std::vector<geometry_msgs::msg::Point> current_polygon_vertices_;

  /// 当前创建的多边形的名称
  std::string current_polygon_name_;

  /// 当前创建的多边形的类型
  AreaType current_polygon_type_;

  /// 用于快速按名称查找多边形的索引
  std::unordered_map<std::string, size_t> name_to_index_;

  // ==================== 私有辅助函数 ====================

  /**
   * @brief 重建名称索引
   *
   * 在添加或删除多边形时调用，维护 name_to_index_ 的正确性
   */
  void rebuildNameIndex();

 private:
};

}  // namespace polygon_manager

#endif  // POLYGON_MANAGER__POLYGON_MANAGER_HPP_
