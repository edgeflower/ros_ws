#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

namespace rm_keepout_costmap_layer
{

struct ForbiddenArea
{
  std::string name;
  std::vector<geometry_msgs::msg::Point> polygon;
};

class PolygonKeepoutLayer : public nav2_costmap_2d::Layer
{
public:
  PolygonKeepoutLayer();
  ~PolygonKeepoutLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  bool isClearable() override;
  void reset() override;

private:
  void loadYaml();
  void reloadTimerCallback();
  bool parseAreas(const YAML::Node & yaml_node);
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  static bool pointInPolygon(double wx, double wy, const std::vector<geometry_msgs::msg::Point> & polygon);

  std::recursive_mutex mutex_;
  std::vector<ForbiddenArea> forbidden_areas_;
  std::string yaml_path_;
  std::string areas_frame_;
  std::string costmap_global_frame_;
  bool areas_frame_matches_costmap_;
  unsigned char keepout_cost_;
  double reload_interval_sec_;
  double huge_area_warn_threshold_;
  time_t last_mtime_;
  rclcpp::Logger logger_{rclcpp::get_logger("polygon_keepout_layer")};
  rclcpp::TimerBase::SharedPtr reload_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace rm_keepout_costmap_layer
