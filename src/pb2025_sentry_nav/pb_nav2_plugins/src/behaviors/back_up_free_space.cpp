// Copyright 2024 Polaris Xia
// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <limits>

namespace pb_nav2_behaviors
{

void BackUpFreeSpace::onConfigure()
{
  nav2_behaviors::DriveOnHeading<BackUpAction>::onConfigure();
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);

  nav2_util::declare_parameter_if_not_declared(
    node, "inscribed_radius", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, "min_escape_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, "obstacle_cost_threshold", rclcpp::ParameterValue(253));

  node->get_parameter("inscribed_radius", inscribed_radius_);
  node->get_parameter("min_escape_distance", min_escape_distance_);
  node->get_parameter("obstacle_cost_threshold", obstacle_cost_threshold_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  nav2_behaviors::DriveOnHeading<BackUpAction>::onCleanup();
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
    return nav2_behaviors::Status::FAILED;
  }

  // get costmap
  auto costmap = result.get()->map;
  cached_costmap_ = costmap;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  // get current pose
  geometry_msgs::msg::Pose2D pose;
  pose.x = initial_pose_.pose.position.x;
  pose.y = initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  // Detect if robot or its footprint is in a danger zone
  starting_in_danger_ = !isPositionFree(pose.x, pose.y);

  // Find the best direction to back up
  auto dir_result = findBestDirection(costmap, pose, -M_PI, M_PI, max_radius_, M_PI / 32.0);

  if (!dir_result.found) {
    RCLCPP_WARN(logger_, "No valid escape direction found");
    return nav2_behaviors::Status::FAILED;
  }

  // Store speed for potential re-search
  command_speed_ = command->speed;
  penetration_distance_ = dir_result.penetration_distance;
  re_search_count_ = 0;
  global_start_time_ = clock_->now();
  last_failed_angle_ = std::numeric_limits<float>::max();

  // Desperate mode: small nudge in the least-bad direction
  if (dir_result.desperate) {
    command_x_ = 0.15f;
    starting_in_danger_ = true;
    twist_x_ = std::cos(dir_result.best_angle) * command_speed_;
    twist_y_ = std::sin(dir_result.best_angle) * command_speed_;
    command_time_allowance_ = command->time_allowance;
    end_time_ = clock_->now() + command_time_allowance_;

    if (!nav2_util::getCurrentPose(
          initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
      RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "Desperate nudge 0.15m at angle %.2f", dir_result.best_angle);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // Compute actual escape distance: at least min_escape, at most clear_distance
  const float cmd_dist = static_cast<float>(std::fabs(command->target.x));
  const float min_needed = dir_result.penetration_distance + static_cast<float>(min_escape_distance_);
  const float max_safe = dir_result.penetration_distance + dir_result.clear_distance;
  const float actual_distance = std::min(std::max(cmd_dist, min_needed), max_safe);

  // Calculate move command
  twist_x_ = std::cos(dir_result.best_angle) * command_speed_;
  twist_y_ = std::sin(dir_result.best_angle) * command_speed_;
  command_x_ = actual_distance;
  command_time_allowance_ = command->time_allowance;

  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }
  RCLCPP_WARN(
    logger_, "backing up %f meters towards free space at angle %f (clear: %f)",
    command_x_, dir_result.best_angle, dir_result.clear_distance);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  // Global timeout: prevents infinite re-search loops
  if ((clock_->now() - global_start_time_).seconds() > GLOBAL_TIMEOUT) {
    stopRobot();
    RCLCPP_WARN(logger_, "Global timeout (%.0fs) exceeded", GLOBAL_TIMEOUT);
    return nav2_behaviors::Status::FAILED;
  }

  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = twist_y_;
  cmd_vel->linear.x = twist_x_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  // Early termination: if robot AND its footprint are now in free space, stop
  if (distance > 0.10f && isPositionFree(pose.x, pose.y)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Escaped to free space after %.2f meters", distance);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // Stuck detection: velocity commands sent but robot hasn't moved
  const double elapsed = command_time_allowance_.seconds() - time_remaining.seconds();
  const bool physically_stuck = (elapsed > 1.0) && (distance < 0.02f);

  // Dynamic danger check: use current position, not initial state
  const bool currently_in_danger = !isPositionFree(pose.x, pose.y);

  // Determine if re-search is needed
  bool need_research = physically_stuck;

  if (!physically_stuck) {
    if (currently_in_danger) {
      // Relaxed check during escape: only block on truly lethal ahead
      const float dir_x = static_cast<float>(twist_x_ / command_speed_);
      const float dir_y = static_cast<float>(twist_y_ / command_speed_);
      const float ahead_x = pose.x + static_cast<float>(inscribed_radius_) * dir_x;
      const float ahead_y = pose.y + static_cast<float>(inscribed_radius_) * dir_y;
      need_research = costAtPosition(ahead_x, ahead_y) >= 254;
    } else {
      need_research = !isCollisionFree(distance, cmd_vel.get(), pose);
    }
  }

  if (need_research) {
    stopRobot();

    // Record failed direction before re-searching
    last_failed_angle_ = std::atan2(static_cast<float>(twist_y_), static_cast<float>(twist_x_));

    if (++re_search_count_ > 5) {
      RCLCPP_ERROR(logger_, "Re-search limit reached (%d), giving up", re_search_count_);
      return nav2_behaviors::Status::FAILED;
    }

    RCLCPP_WARN(logger_, "%s - re-search #%d",
      physically_stuck ? "Physically stuck" : "Collision ahead", re_search_count_);

    auto req = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    auto res = costmap_client_->async_send_request(req);
    if (res.wait_for(std::chrono::milliseconds(500)) == std::future_status::timeout) {
      RCLCPP_ERROR(logger_, "Re-search: costmap service timeout");
      return nav2_behaviors::Status::FAILED;
    }

    geometry_msgs::msg::Pose2D re_pose;
    re_pose.x = current_pose.pose.position.x;
    re_pose.y = current_pose.pose.position.y;
    re_pose.theta = tf2::getYaw(current_pose.pose.orientation);

    auto re_result =
      findBestDirection(res.get()->map, re_pose, -M_PI, M_PI, max_radius_, M_PI / 32.0, last_failed_angle_);

    if (!re_result.found) {
      RCLCPP_WARN(logger_, "Re-search: no valid direction found");
      return nav2_behaviors::Status::FAILED;
    }

    const float remaining = std::fabs(command_x_) - distance;
    const float re_min_needed =
      re_result.penetration_distance + static_cast<float>(min_escape_distance_);
    const float re_max_safe =
      re_result.penetration_distance + re_result.clear_distance;
    const float new_target = std::min(std::max(remaining, re_min_needed), re_max_safe);

    twist_x_ = std::cos(re_result.best_angle) * command_speed_;
    twist_y_ = std::sin(re_result.best_angle) * command_speed_;
    command_x_ = new_target;
    penetration_distance_ = re_result.penetration_distance;
    initial_pose_ = current_pose;
    end_time_ = clock_->now() + command_time_allowance_;

    RCLCPP_WARN(
      logger_, "Re-search #%d: angle %.2f, dist %.2f, penetrate %.2f",
      re_search_count_, re_result.best_angle, new_target, re_result.penetration_distance);
    return nav2_behaviors::Status::RUNNING;
  }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

unsigned char BackUpFreeSpace::costAtPosition(float x, float y) const
{
  const float resolution = cached_costmap_.metadata.resolution;
  const float origin_x = cached_costmap_.metadata.origin.position.x;
  const float origin_y = cached_costmap_.metadata.origin.position.y;
  const int size_x = cached_costmap_.metadata.size_x;
  const int size_y = cached_costmap_.metadata.size_y;

  int i = static_cast<int>((x - origin_x) / resolution);
  int j = static_cast<int>((y - origin_y) / resolution);

  if (i < 0 || i >= size_x || j < 0 || j >= size_y) return 255;
  return cached_costmap_.data[i + j * size_x];
}

bool BackUpFreeSpace::isPositionFree(float x, float y) const
{
  if (costAtPosition(x, y) >= obstacle_cost_threshold_) return false;
  // Check footprint circle at inscribed_radius (16 sample points)
  for (float a = 0; a < 2.0f * static_cast<float>(M_PI); a += static_cast<float>(M_PI) / 8.0f) {
    float cx = x + static_cast<float>(inscribed_radius_) * std::cos(a);
    float cy = y + static_cast<float>(inscribed_radius_) * std::sin(a);
    if (costAtPosition(cx, cy) >= obstacle_cost_threshold_) return false;
  }
  return true;
}

DirectionResult BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment, float excluded_angle)
{
  DirectionResult result;

  const float resolution = costmap.metadata.resolution;
  const float origin_x = costmap.metadata.origin.position.x;
  const float origin_y = costmap.metadata.origin.position.y;
  const int size_x = costmap.metadata.size_x;
  const int size_y = costmap.metadata.size_y;
  const float map_max_x = origin_x + size_x * resolution;
  const float map_max_y = origin_y + size_y * resolution;

  // Phase 1: Collect per-angle safety and max safe ray length
  struct AngleInfo
  {
    float angle;
    bool safe;
    float max_safe_r;
  };
  std::vector<AngleInfo> infos;

  for (float angle = start_angle; angle <= end_angle + 1e-6f; angle += angle_increment) {
    bool is_safe = true;
    float max_r = static_cast<float>(inscribed_radius_);

    for (float r = static_cast<float>(inscribed_radius_); r <= radius; r += resolution) {
      float x = pose.x + r * std::cos(angle);
      float y = pose.y + r * std::sin(angle);

      if (x < origin_x || x >= map_max_x || y < origin_y || y >= map_max_y) {
        is_safe = false;
        break;
      }

      int i = static_cast<int>((x - origin_x) / resolution);
      int j = static_cast<int>((y - origin_y) / resolution);

      if (i < 0 || i >= size_x || j < 0 || j >= size_y ||
          costmap.data[i + j * size_x] >= obstacle_cost_threshold_) {
        is_safe = false;
        break;
      }
      max_r = r;
    }

    infos.push_back({angle, is_safe, max_r});
  }

  const int N = static_cast<int>(infos.size());
  if (N == 0) return result;

  // Phase 2: Find longest safe sector using circular scan (handles ±π wrap)
  int best_len = 0, best_start = 0;
  int cur_len = 0, cur_start = 0;

  for (int i = 0; i < 2 * N; i++) {
    if (infos[i % N].safe) {
      if (cur_len == 0) cur_start = i;
      cur_len++;
    } else {
      if (cur_len > best_len) {
        best_len = cur_len;
        best_start = cur_start;
      }
      cur_len = 0;
    }
  }
  if (cur_len > best_len) {
    best_len = cur_len;
    best_start = cur_start;
  }
  if (best_len > N) best_len = N;

  if (best_len == 0) {
    // Fallback: penetration search - find directions that lead to free space
    // through high-cost zones (inflation, noise, stale data)
    RCLCPP_WARN(logger_, "No fully safe sector - attempting penetration search");

    float best_score = -1.0f;

    for (int i = 0; i < N; i++) {
      const float angle = infos[i].angle;
      float first_free_r = -1.0f;
      float free_end_r = -1.0f;
      bool in_free = false;

      for (float r = 0; r <= radius; r += resolution) {
        float x = pose.x + r * std::cos(angle);
        float y = pose.y + r * std::sin(angle);

        if (x < origin_x || x >= map_max_x || y < origin_y || y >= map_max_y) {
          if (in_free) break;
          continue;
        }

        int mi = static_cast<int>((x - origin_x) / resolution);
        int mj = static_cast<int>((y - origin_y) / resolution);

        if (mi < 0 || mi >= size_x || mj < 0 || mj >= size_y) {
          if (in_free) break;
          continue;
        }

        if (costmap.data[mi + mj * size_x] < obstacle_cost_threshold_) {
          if (first_free_r < 0) first_free_r = r;
          in_free = true;
          free_end_r = r;
        } else if (in_free) {
          break;
        }
      }

      if (first_free_r < 0) continue;

      const float free_depth = free_end_r - first_free_r + resolution;
      if (free_depth < static_cast<float>(min_escape_distance_)) continue;

      // Score: prefer short penetration + long free depth, penalize failed directions
      float score = free_depth / (first_free_r + resolution);
      if (excluded_angle < std::numeric_limits<float>::max()) {
        float angle_diff = std::fabs(std::atan2(std::sin(angle - excluded_angle),
                                                 std::cos(angle - excluded_angle)));
        if (angle_diff < 0.52f) score *= 0.1f;  // ±30° penalty
      }
      if (score > best_score) {
        best_score = score;
        result.found = true;
        result.best_angle = angle;
        result.clear_distance = free_depth;
        result.penetration_distance = first_free_r;
      }
    }

    if (result.found) {
      RCLCPP_WARN(
        logger_, "Penetration search: angle %.2f, penetrate %.2f, clear %.2f",
        result.best_angle, result.penetration_distance, result.clear_distance);
      if (visualize_) {
        visualize(pose, radius, result.best_angle, result.best_angle);
      }
      return result;
    }

    // Final fallback: no free space at all. Find direction with lowest average cost.
    RCLCPP_WARN(logger_, "No free corridor - attempting lowest-cost direction");
    float best_avg_cost = std::numeric_limits<float>::max();

    for (int i = 0; i < N; i++) {
      const float angle = infos[i].angle;
      float total_cost = 0.0f;
      int count = 0;

      for (float r = 0; r <= radius; r += resolution) {
        float x = pose.x + r * std::cos(angle);
        float y = pose.y + r * std::sin(angle);

        if (x < origin_x || x >= map_max_x || y < origin_y || y >= map_max_y) continue;

        int mi = static_cast<int>((x - origin_x) / resolution);
        int mj = static_cast<int>((y - origin_y) / resolution);

        if (mi < 0 || mi >= size_x || mj < 0 || mj >= size_y) continue;

        total_cost += costmap.data[mi + mj * size_x];
        count++;
      }

      if (count > 0) {
        float avg_cost = total_cost / static_cast<float>(count);
        // Penalize failed directions
        if (excluded_angle < std::numeric_limits<float>::max()) {
          float angle_diff = std::fabs(std::atan2(std::sin(angle - excluded_angle),
                                                   std::cos(angle - excluded_angle)));
          if (angle_diff < 0.52f) avg_cost += 50.0f;  // ±30° penalty
        }
        if (avg_cost < best_avg_cost) {
          best_avg_cost = avg_cost;
          result.found = true;
          result.best_angle = angle;
          result.desperate = true;
        }
      }
    }

    if (result.found) {
      RCLCPP_WARN(logger_, "Desperate move: angle %.2f, avg_cost %.1f", result.best_angle, best_avg_cost);
    }

    return result;
  }

  // Phase 3: Compute center angle and clear distance
  float sum_cos = 0.0f, sum_sin = 0.0f;
  float min_clear = std::numeric_limits<float>::max();
  float vis_start_angle = 0.0f, vis_end_angle = 0.0f;

  for (int k = best_start; k < best_start + best_len; k++) {
    const auto & info = infos[k % N];
    sum_cos += std::cos(info.angle);
    sum_sin += std::sin(info.angle);
    min_clear = std::min(min_clear, info.max_safe_r);
    if (k == best_start) vis_start_angle = info.angle;
    vis_end_angle = info.angle;
  }

  result.found = true;
  result.best_angle = std::atan2(sum_sin, sum_cos);
  result.clear_distance = min_clear;

  if (visualize_) {
    visualize(pose, radius, vis_start_angle, vis_end_angle);
  }

  return result;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  geometry_msgs::msg::Pose2D pose, float radius, float first_safe_angle, float last_unsafe_angle)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = clock_->now();
  sector_marker.ns = "direction";
  sector_marker.id = 0;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 1.0f;
  sector_marker.color.b = 0.0f;
  sector_marker.color.a = 0.2f;

  const float angle_step = 0.05f;
  for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
    const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = 0.0;

    geometry_msgs::msg::Point p1;
    p1.x = pose.x + radius * std::cos(angle);
    p1.y = pose.y + radius * std::sin(angle);
    p1.z = 0.0;

    geometry_msgs::msg::Point p2;
    p2.x = pose.x + radius * std::cos(next_angle);
    p2.y = pose.y + radius * std::sin(next_angle);
    p2.z = 0.0;

    sector_marker.points.push_back(origin);
    sector_marker.points.push_back(p1);
    sector_marker.points.push_back(p2);
  }
  markers.markers.push_back(sector_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = clock_->now();
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  markers.markers.push_back(create_arrow(first_safe_angle, 1, 0.0f, 0.0f, 1.0f));
  markers.markers.push_back(create_arrow(last_unsafe_angle, 2, 0.0f, 0.0f, 1.0f));

  const float best_angle = (first_safe_angle + last_unsafe_angle) / 2.0f;
  markers.markers.push_back(create_arrow(best_angle, 3, 0.0f, 1.0f, 0.0f));

  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
