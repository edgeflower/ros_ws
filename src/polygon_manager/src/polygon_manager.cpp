#include <algorithm>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include "polygon_manager/polygon_manager.hpp"
#include "polygon_manager/geometry_utils.hpp"

namespace polygon_manager {

/**
 * PolygonManager 构造函数
 * 初始化空的多边形管理器
 */
PolygonManager::PolygonManager()
    : next_polygon_id_(1),
      current_polygon_type_(AreaType::FORBIDDEN_AREA) {
}

// ==================== 多边形管理实现 ====================

/**
 * 添加新的多边形区域
 * 
 * 流程：
 * 1. 验证多边形有效性（至少 3 个顶点）
 * 2. 检查名称唯一性
 * 3. 创建 PolygonArea 并分配 ID
 * 4. 添加到管理列表
 * 5. 更新索引
 */
bool PolygonManager::addPolygon(
    const std::string& name,
    AreaType type,
    const std::vector<geometry_msgs::msg::Point>& vertices) {
  
  // 验证多边形有效性
  if (!GeometryUtils::isValidPolygon(vertices)) {
    RCLCPP_WARN(rclcpp::get_logger("PolygonManager"),
                "Invalid polygon: need at least 3 vertices");
    return false;
  }

  // 检查名称唯一性
  if (name_to_index_.find(name) != name_to_index_.end()) {
    RCLCPP_WARN(rclcpp::get_logger("PolygonManager"),
                "Polygon name '%s' already exists", name.c_str());
    return false;
  }

  // 创建新的多边形
  PolygonArea polygon;
  polygon.id = next_polygon_id_++;
  polygon.name = name;
  polygon.type = type;
  polygon.vertices = vertices;

  // 添加到列表
  name_to_index_[name] = polygons_.size();
  polygons_.push_back(polygon);

  RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
              "Added polygon '%s' with ID %u", name.c_str(), polygon.id);

  return true;
}

/**
 * 删除指定 ID 的多边形
 */
bool PolygonManager::deletePolygon(uint32_t polygon_id) {
  auto it = std::find_if(polygons_.begin(), polygons_.end(),
      [polygon_id](const PolygonArea& p) { return p.id == polygon_id; });

  if (it == polygons_.end()) {
    RCLCPP_WARN(rclcpp::get_logger("PolygonManager"),
                "Polygon with ID %u not found", polygon_id);
    return false;
  }

  std::string deleted_name = it->name;
  polygons_.erase(it);
  rebuildNameIndex();

  RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
              "Deleted polygon '%s'", deleted_name.c_str());

  return true;
}

/**
 * 按名称删除多边形
 */
bool PolygonManager::deletePolygonByName(const std::string& name) {
  auto it = name_to_index_.find(name);
  if (it == name_to_index_.end()) {
    RCLCPP_WARN(rclcpp::get_logger("PolygonManager"),
                "Polygon '%s' not found", name.c_str());
    return false;
  }

  size_t index = it->second;
  polygons_.erase(polygons_.begin() + index);
  rebuildNameIndex();

  RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
              "Deleted polygon '%s'", name.c_str());

  return true;
}

/**
 * 清空所有多边形
 */
void PolygonManager::clearAllPolygons() {
  polygons_.clear();
  name_to_index_.clear();
  RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
              "Cleared all polygons");
}

/**
 * 开始交互式创建新多边形
 */
void PolygonManager::startCreatingPolygon(
    const std::string& name,
    AreaType type) {
  current_polygon_vertices_.clear();
  current_polygon_name_ = name;
  current_polygon_type_ = type;

  RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
              "Started creating polygon '%s'", name.c_str());
}

/**
 * 向当前创建的多边形添加顶点
 * 
 * 每次点击 RViz 中的点时调用
 */
size_t PolygonManager::addVertexToCurrentPolygon(
    const geometry_msgs::msg::Point& point) {
  current_polygon_vertices_.push_back(point);

  size_t vertex_count = current_polygon_vertices_.size();
  RCLCPP_DEBUG(rclcpp::get_logger("PolygonManager"),
               "Added vertex to current polygon. Total vertices: %zu", vertex_count);

  return vertex_count;
}

/**
 * 闭合当前创建的多边形
 * 
 * 检查顶点数量，如果有效则将其添加到管理列表
 */
bool PolygonManager::closeCurrentPolygon() {
  if (current_polygon_vertices_.size() < 3) {
    RCLCPP_WARN(rclcpp::get_logger("PolygonManager"),
                "Cannot close polygon: need at least 3 vertices, got %zu",
                current_polygon_vertices_.size());
    return false;
  }

  bool success = addPolygon(
      current_polygon_name_,
      current_polygon_type_,
      current_polygon_vertices_);

  if (success) {
    current_polygon_vertices_.clear();
    current_polygon_name_ = "";
  }

  return success;
}

/**
 * 取消当前多边形的创建
 */
void PolygonManager::cancelCurrentPolygon() {
  current_polygon_vertices_.clear();
  current_polygon_name_ = "";
  RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
              "Cancelled current polygon creation");
}

// ==================== 敌方检测实现 ====================

/**
 * 检测敌方是否在禁区内
 */
bool PolygonManager::isEnemyInForbiddenArea(
    const geometry_msgs::msg::PoseStamped& enemy_pose) const {
  for (const auto& polygon : polygons_) {
    if (polygon.type == AreaType::FORBIDDEN_AREA) {
      if (GeometryUtils::isPointInPolygon(
          enemy_pose.pose.position.x,
          enemy_pose.pose.position.y,
          polygon.vertices)) {
        return true;
      }
    }
  }
  return false;
}

/**
 * 检测敌方所在的所有区域类型
 */
std::vector<AreaType> PolygonManager::getEnemyAreas(
    const geometry_msgs::msg::PoseStamped& enemy_pose) const {
  std::vector<AreaType> areas;

  for (const auto& polygon : polygons_) {
    if (GeometryUtils::isPointInPolygon(
        enemy_pose.pose.position.x,
        enemy_pose.pose.position.y,
        polygon.vertices)) {
      areas.push_back(polygon.type);
    }
  }

  return areas;
}

/**
 * 检测敌方所在的所有区域名称
 */
std::vector<std::string> PolygonManager::getEnemyAreaNames(
    const geometry_msgs::msg::PoseStamped& enemy_pose) const {
  std::vector<std::string> area_names;

  for (const auto& polygon : polygons_) {
    if (GeometryUtils::isPointInPolygon(
        enemy_pose.pose.position.x,
        enemy_pose.pose.position.y,
        polygon.vertices)) {
      area_names.push_back(polygon.name);
    }
  }

  return area_names;
}

/**
 * 生成敌方状态字符串
 * 
 * 返回格式：
 * - 如果在禁区内：返回第一个禁区的名称
 * - 如果在其他区域：返回所有区域的名称
 * - 如果都不在：返回 "enemy_area_unknown"
 */
std::string PolygonManager::generateEnemyStateString(
    const geometry_msgs::msg::PoseStamped& enemy_pose) const {
  // 首先检查禁区
  for (const auto& polygon : polygons_) {
    if (polygon.type == AreaType::FORBIDDEN_AREA) {
      if (GeometryUtils::isPointInPolygon(
          enemy_pose.pose.position.x,
          enemy_pose.pose.position.y,
          polygon.vertices)) {
        return "enemy_in_" + polygon.name;
      }
    }
  }

  // 检查其他区域
  auto areas = getEnemyAreaNames(enemy_pose);
  if (!areas.empty()) {
    return "enemy_in_" + areas[0];
  }

  return "enemy_area_unknown";
}

// ==================== 数据查询实现 ====================

/**
 * 按 ID 查找多边形
 */
const PolygonArea* PolygonManager::getPolygonById(uint32_t polygon_id) const {
  auto it = std::find_if(polygons_.begin(), polygons_.end(),
      [polygon_id](const PolygonArea& p) { return p.id == polygon_id; });

  return (it != polygons_.end()) ? &(*it) : nullptr;
}

/**
 * 按名称查找多边形
 */
const PolygonArea* PolygonManager::getPolygonByName(
    const std::string& name) const {
  auto it = name_to_index_.find(name);
  if (it == name_to_index_.end()) {
    return nullptr;
  }

  size_t index = it->second;
  if (index >= polygons_.size()) {
    return nullptr;
  }

  return &polygons_[index];
}

// ==================== 可视化实现 ====================

/**
 * 生成所有多边形的 MarkerArray
 */
visualization_msgs::msg::MarkerArray PolygonManager::getMarkerArray() const {
  return MarkerUtils::createMarkerArrayFromPolygons(polygons_);
}

/**
 * 生成当前创建中的多边形的 Marker
 * 
 * 用于实时显示用户正在编辑的多边形
 */
visualization_msgs::msg::Marker PolygonManager::getCurrentPolygonMarker() const {
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time(0);
  
  marker.ns = "current_polygon";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // 线宽
  marker.scale.x = 0.01;

  // 使用不同颜色表示当前编辑中的多边形（白色）
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.8;

  // 添加当前顶点
  for (const auto& vertex : current_polygon_vertices_) {
    geometry_msgs::msg::Point p;
    p.x = vertex.x;
    p.y = vertex.y;
    p.z = 0.0;
    marker.points.push_back(p);
  }

  return marker;
}

// ==================== 配置管理实现 ====================

/**
 * 从 YAML 文件加载配置
 * 
 * 文件格式示例：
 * areas:
 *   - name: forbidden_zone_1
 *     type: forbidden
 *     points:
 *       - [1.0, 1.0]
 *       - [5.0, 1.0]
 *       - [5.0, 4.0]
 */
bool PolygonManager::loadFromYaml(const std::string& yaml_file_path) {
  try {
    YAML::Node config = YAML::LoadFile(yaml_file_path);

    if (!config["areas"]) {
      RCLCPP_WARN(rclcpp::get_logger("PolygonManager"),
                  "No 'areas' field in YAML file");
      return false;
    }

    clearAllPolygons();

    for (const auto& area_node : config["areas"]) {
      std::string name = area_node["name"].as<std::string>();
      std::string type_str = area_node["type"].as<std::string>();
      AreaType type = stringToAreaType(type_str);

      std::vector<geometry_msgs::msg::Point> vertices;
      for (const auto& point_node : area_node["points"]) {
        geometry_msgs::msg::Point point;
        point.x = point_node[0].as<double>();
        point.y = point_node[1].as<double>();
        point.z = 0.0;
        vertices.push_back(point);
      }

      addPolygon(name, type, vertices);
    }

    RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
                "Loaded %zu polygons from '%s'",
                polygons_.size(), yaml_file_path.c_str());

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("PolygonManager"),
                 "Failed to load YAML file: %s", e.what());
    return false;
  }
}

/**
 * 保存当前配置到 YAML 文件
 */
bool PolygonManager::saveToYaml(const std::string& yaml_file_path) const {
  try {
    YAML::Node root;
    YAML::Node areas_node = YAML::Node(YAML::NodeType::Sequence);

    for (const auto& polygon : polygons_) {
      YAML::Node area_node;
      area_node["name"] = polygon.name;
      area_node["type"] = areaTypeToString(polygon.type);

      YAML::Node points_node = YAML::Node(YAML::NodeType::Sequence);
      for (const auto& vertex : polygon.vertices) {
        YAML::Node point;
        point.push_back(vertex.x);
        point.push_back(vertex.y);
        points_node.push_back(point);
      }
      area_node["points"] = points_node;

      areas_node.push_back(area_node);
    }

    root["areas"] = areas_node;

    std::ofstream fout(yaml_file_path);
    fout << root;
    fout.close();

    RCLCPP_INFO(rclcpp::get_logger("PolygonManager"),
                "Saved %zu polygons to '%s'",
                polygons_.size(), yaml_file_path.c_str());

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("PolygonManager"),
                 "Failed to save YAML file: %s", e.what());
    return false;
  }
}

// ==================== 私有辅助函数实现 ====================

/**
 * 重建名称索引
 */
void PolygonManager::rebuildNameIndex() {
  name_to_index_.clear();
  for (size_t i = 0; i < polygons_.size(); ++i) {
    name_to_index_[polygons_[i].name] = i;
  }
}

/**
 * 字符串转换为 AreaType
 */
AreaType PolygonManager::stringToAreaType(
    const std::string& type_str) {
  if (type_str == "forbidden") {
    return AreaType::FORBIDDEN_AREA;
  } else if (type_str == "protect") {
    return AreaType::PROTECT_AREA;
  } else if (type_str == "patrol") {
    return AreaType::PATROL_AREA;
  } else if (type_str == "attack") {
    return AreaType::ATTACK_AREA;
  }
  return AreaType::FORBIDDEN_AREA;  // 默认
}

/**
 * AreaType 转换为字符串
 */
std::string PolygonManager::areaTypeToString(AreaType type) {
  switch (type) {
    case AreaType::FORBIDDEN_AREA:
      return "forbidden";
    case AreaType::PROTECT_AREA:
      return "protect";
    case AreaType::PATROL_AREA:
      return "patrol";
    case AreaType::ATTACK_AREA:
      return "attack";
    default:
      return "forbidden";
  }
}

}  // namespace polygon_manager
