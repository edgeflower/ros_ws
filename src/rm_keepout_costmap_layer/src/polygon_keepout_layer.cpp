#include "rm_keepout_costmap_layer/polygon_keepout_layer.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <sys/stat.h>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rm_keepout_costmap_layer::PolygonKeepoutLayer, nav2_costmap_2d::Layer)

namespace fs = std::filesystem;

namespace rm_keepout_costmap_layer
{

PolygonKeepoutLayer::PolygonKeepoutLayer()
: areas_frame_matches_costmap_(false),
  keepout_cost_(254),
  reload_interval_sec_(2.0),
  huge_area_warn_threshold_(20.0),
  last_mtime_(0)
{
}

bool PolygonKeepoutLayer::isClearable()
{
  return false;
}

void PolygonKeepoutLayer::reset()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  current_ = false;
  loadYaml();
  current_ = true;
}

void PolygonKeepoutLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("PolygonKeepoutLayer: unable to lock node");
  }
  logger_ = node->get_logger();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("yaml_path", rclcpp::ParameterValue(std::string("/home/sentry/areas.yaml")));
  declareParameter("keepout_cost", rclcpp::ParameterValue(254));
  declareParameter("reload_interval_sec", rclcpp::ParameterValue(2.0));
  declareParameter("areas_frame", rclcpp::ParameterValue(std::string("map")));
  declareParameter("huge_area_warn_threshold", rclcpp::ParameterValue(20.0));

  yaml_path_ = node->get_parameter(name_ + ".yaml_path").as_string();
  int configured_keepout_cost = static_cast<int>(node->get_parameter(name_ + ".keepout_cost").as_int());
  int clamped_keepout_cost = std::clamp(configured_keepout_cost, 0, 254);
  if (configured_keepout_cost != clamped_keepout_cost) {
    RCLCPP_WARN(
      logger_, "keepout_cost=%d is outside [0, 254], clamped to %d",
      configured_keepout_cost, clamped_keepout_cost);
  }
  keepout_cost_ = static_cast<unsigned char>(clamped_keepout_cost);
  reload_interval_sec_ = node->get_parameter(name_ + ".reload_interval_sec").as_double();
  areas_frame_ = node->get_parameter(name_ + ".areas_frame").as_string();
  huge_area_warn_threshold_ =
    node->get_parameter(name_ + ".huge_area_warn_threshold").as_double();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();

  costmap_global_frame_ = layered_costmap_->getGlobalFrameID();
  areas_frame_matches_costmap_ = areas_frame_ == costmap_global_frame_;
  if (!areas_frame_matches_costmap_) {
    RCLCPP_ERROR(
      logger_,
      "PolygonKeepoutLayer only supports polygons already in the costmap frame: "
      "areas_frame='%s', costmap_global_frame='%s'. Keepout costs will not be written.",
      areas_frame_.c_str(), costmap_global_frame_.c_str());
  } else {
    RCLCPP_INFO(
      logger_, "PolygonKeepoutLayer areas_frame='%s' matches costmap_global_frame='%s'",
      areas_frame_.c_str(), costmap_global_frame_.c_str());
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return dynamicParametersCallback(parameters);
    });

  loadYaml();

  reload_timer_ = node->create_wall_timer(
    std::chrono::duration<double>(reload_interval_sec_),
    [this]() { reloadTimerCallback(); });

  RCLCPP_INFO(
    logger_, "PolygonKeepoutLayer initialized, yaml_path=%s, enabled=%s, keepout_cost=%u",
    yaml_path_.c_str(), enabled_ ? "true" : "false", static_cast<unsigned int>(keepout_cost_));
}

rcl_interfaces::msg::SetParametersResult PolygonKeepoutLayer::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const std::string & param_name = parameter.get_name();
    if (param_name == name_ + ".enabled" || param_name == "enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = name_ + ".enabled must be a bool";
        return result;
      }
    } else if (param_name == name_ + ".keepout_cost" || param_name == "keepout_cost") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = name_ + ".keepout_cost must be an integer in [0, 254]";
        return result;
      }
    }
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (const auto & parameter : parameters) {
    const std::string & param_name = parameter.get_name();
    if (param_name == name_ + ".enabled" || param_name == "enabled") {
      enabled_ = parameter.as_bool();
      current_ = true;
      RCLCPP_INFO(
        logger_, "Runtime parameter update: %s.enabled=%s",
        name_.c_str(), enabled_ ? "true" : "false");
    } else if (param_name == name_ + ".keepout_cost" || param_name == "keepout_cost") {
      int requested_cost = static_cast<int>(parameter.as_int());
      int clamped_cost = std::clamp(requested_cost, 0, 254);
      if (requested_cost != clamped_cost) {
        RCLCPP_WARN(
          logger_, "Runtime parameter update: %s.keepout_cost=%d outside [0, 254], clamped to %d",
          name_.c_str(), requested_cost, clamped_cost);
      }
      keepout_cost_ = static_cast<unsigned char>(clamped_cost);
      current_ = true;
      RCLCPP_INFO(
        logger_, "Runtime parameter update: %s.keepout_cost=%u",
        name_.c_str(), static_cast<unsigned int>(keepout_cost_));
    }
  }

  return result;
}

void PolygonKeepoutLayer::loadYaml()
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (!fs::exists(yaml_path_)) {
    RCLCPP_WARN(logger_, "YAML file does not exist: %s", yaml_path_.c_str());
    return;
  }

  try {
    YAML::Node yaml_node = YAML::LoadFile(yaml_path_);

    // Record mtime
    struct stat file_stat;
    if (stat(yaml_path_.c_str(), &file_stat) == 0) {
      last_mtime_ = file_stat.st_mtime;
    }

    if (parseAreas(yaml_node)) {
      RCLCPP_INFO(
        logger_, "Loaded %zu forbidden area(s) from %s",
        forbidden_areas_.size(), yaml_path_.c_str());
      current_ = true;
    }
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(logger_, "Failed to parse YAML %s: %s", yaml_path_.c_str(), e.what());
  }
}

bool PolygonKeepoutLayer::parseAreas(const YAML::Node & yaml_node)
{
  if (!yaml_node["areas"]) {
    RCLCPP_WARN(logger_, "YAML has no 'areas' key");
    return false;
  }

  const auto & areas = yaml_node["areas"];
  if (!areas.IsSequence()) {
    RCLCPP_ERROR(logger_, "'areas' is not a sequence");
    return false;
  }

  std::vector<ForbiddenArea> new_areas;

  for (size_t i = 0; i < areas.size(); ++i) {
    const auto & area = areas[i];

    if (!area["type"]) {
      RCLCPP_WARN(logger_, "Area #%zu missing 'type', skipping", i);
      continue;
    }

    std::string type = area["type"].as<std::string>();
    if (type != "forbidden") {
      continue;
    }

    ForbiddenArea fa;
    fa.name = area["name"] ? area["name"].as<std::string>() : ("area_" + std::to_string(i));

    const auto & points = area["points"];
    if (!points || !points.IsSequence() || points.size() < 3) {
      RCLCPP_WARN(logger_, "Area '%s' has < 3 points, skipping", fa.name.c_str());
      continue;
    }

    for (size_t j = 0; j < points.size(); ++j) {
      const auto & pt = points[j];
      if (!pt.IsSequence() || pt.size() < 2) {
        RCLCPP_WARN(logger_, "Area '%s' point #%zu invalid format", fa.name.c_str(), j);
        continue;
      }

      double x = pt[0].as<double>();
      double y = pt[1].as<double>();

      // 检查坐标是否有效（不是 NaN 或无穷大）
      if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y)) {
        RCLCPP_WARN(
          logger_, "Area '%s' point #%zu has invalid coordinates (NaN/Inf)",
          fa.name.c_str(), j);
        continue;
      }

      geometry_msgs::msg::Point p;
      p.x = x;
      p.y = y;
      p.z = 0.0;
      fa.polygon.push_back(p);
    }

    if (fa.polygon.size() >= 3) {
      // 输出多边形的 bounding box 用于调试
      double min_x = fa.polygon[0].x, max_x = fa.polygon[0].x;
      double min_y = fa.polygon[0].y, max_y = fa.polygon[0].y;
      for (const auto & pt : fa.polygon) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
      }
      const double bbox_width = max_x - min_x;
      const double bbox_height = max_y - min_y;
      const double bbox_area = bbox_width * bbox_height;

      RCLCPP_INFO(
        logger_, "Forbidden area '%s': %zu points, bbox: [%.2f, %.2f] to [%.2f, %.2f], area=%.2f m^2",
        fa.name.c_str(), fa.polygon.size(), min_x, min_y, max_x, max_y, bbox_area);
      if (bbox_area > huge_area_warn_threshold_) {
        RCLCPP_WARN(
          logger_,
          "Forbidden area '%s' bbox is large: min_x=%.2f, min_y=%.2f, max_x=%.2f, max_y=%.2f, area=%.2f m^2",
          fa.name.c_str(), min_x, min_y, max_x, max_y, bbox_area);
      }
      new_areas.push_back(std::move(fa));
    }
  }

  forbidden_areas_ = std::move(new_areas);
  return true;
}

void PolygonKeepoutLayer::reloadTimerCallback()
{
  struct stat file_stat;
  if (stat(yaml_path_.c_str(), &file_stat) != 0) {
    return;
  }

  if (file_stat.st_mtime != last_mtime_) {
    RCLCPP_INFO(logger_, "YAML modified, reloading: %s", yaml_path_.c_str());
    loadYaml();
  }
}

// 改进的点在多边形内判断算法
// 使用更稳定的 ray casting 实现，避免边界情况
bool PolygonKeepoutLayer::pointInPolygon(
  double wx, double wy, const std::vector<geometry_msgs::msg::Point> & polygon)
{
  const double EPSILON = 1e-10;
  int n = static_cast<int>(polygon.size());
  bool inside = false;

  for (int i = 0, j = n - 1; i < n; j = i++) {
    double xi = polygon[i].x, yi = polygon[i].y;
    double xj = polygon[j].x, yj = polygon[j].y;

    // 检查点是否在边上（边界情况）
    if (std::abs(yj - yi) < EPSILON) {
      // 边是水平的，跳过
      continue;
    }

    // 改进的射线投射条件
    // 避免在顶点处的歧义
    if ((yi <= wy && wy < yj) || (yj <= wy && wy < yi)) {
      // 计算射线与边的交点的 x 坐标
      double x_intersect = (xj - xi) * (wy - yi) / (yj - yi) + xi;

      // 只有当点在交点右侧时才翻转 inside 状态
      if (wx < x_intersect) {
        inside = !inside;
      }
    }
  }

  return inside;
}

void PolygonKeepoutLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (!enabled_ || !areas_frame_matches_costmap_) {
    return;
  }

  for (const auto & area : forbidden_areas_) {
    for (const auto & pt : area.polygon) {
      *min_x = std::min(*min_x, pt.x);
      *min_y = std::min(*min_y, pt.y);
      *max_x = std::max(*max_x, pt.x);
      *max_y = std::max(*max_y, pt.y);
    }
  }
}

void PolygonKeepoutLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  if (!enabled_ || !areas_frame_matches_costmap_ || forbidden_areas_.empty()) {
    return;
  }

  const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
  const int size_y = static_cast<int>(master_grid.getSizeInCellsY());

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(size_x, max_i);
  max_j = std::min(size_y, max_j);

  if (min_i >= max_i || min_j >= max_j) {
    return;
  }

  for (const auto & area : forbidden_areas_) {
    if (area.polygon.size() < 3) {
      continue;
    }

    // Compute polygon bounding box in world coords
    double poly_min_x = std::numeric_limits<double>::max();
    double poly_min_y = std::numeric_limits<double>::max();
    double poly_max_x = -std::numeric_limits<double>::max();
    double poly_max_y = -std::numeric_limits<double>::max();

    for (const auto & pt : area.polygon) {
      poly_min_x = std::min(poly_min_x, pt.x);
      poly_min_y = std::min(poly_min_y, pt.y);
      poly_max_x = std::max(poly_max_x, pt.x);
      poly_max_y = std::max(poly_max_y, pt.y);
    }

    // 检查 bounding box 是否有效
    if (poly_min_x >= poly_max_x || poly_min_y >= poly_max_y) {
      RCLCPP_WARN(logger_, "Area '%s' has invalid bounding box", area.name.c_str());
      continue;
    }

    // Convert bounding box to cell indices, clamped to costmap bounds
    int p_min_i, p_min_j, p_max_i, p_max_j;
    master_grid.worldToMapEnforceBounds(poly_min_x, poly_min_y, p_min_i, p_min_j);
    master_grid.worldToMapEnforceBounds(poly_max_x, poly_max_y, p_max_i, p_max_j);

    // Intersect polygon bbox with the update window
    int start_i = std::max(min_i, p_min_i);
    int start_j = std::max(min_j, p_min_j);
    int end_i = std::min(max_i, p_max_i + 1);
    int end_j = std::min(max_j, p_max_j + 1);

    if (start_i >= end_i || start_j >= end_j) {
      continue;
    }

    // Only scan cells within the bounding box
    for (int i = start_i; i < end_i; ++i) {
      for (int j = start_j; j < end_j; ++j) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);

        if (!pointInPolygon(wx, wy, area.polygon)) {
          continue;
        }

        unsigned char old_cost = master_grid.getCost(i, j);
        if (old_cost < keepout_cost_) {
          master_grid.setCost(i, j, keepout_cost_);
        }
      }
    }
  }
}

}  // namespace rm_keepout_costmap_layer
