// Map Processor Node - Offline/Periodic Map Analysis for Observation Point Generation
// File: map_processor_node.cpp

#include "rm_behavior_tree/map_processor_node.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace rm_behavior_tree
{

MapProcessorNode::MapProcessorNode(const rclcpp::NodeOptions & options)
: Node("map_processor_node", options), first_map_received_(false)
{
    // Declare parameters
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("sample_step", 2.0);
    this->declare_parameter("robot_radius", 0.3);
    this->declare_parameter("ray_count", 36);
    this->declare_parameter("update_rate", 0.1);

    // Distance transform parameters
    this->declare_parameter("min_distance_threshold", 0.5);
    this->declare_parameter("distance_weight", 0.3);

    // NMS parameters
    this->declare_parameter("nms_buffer_radius", 1.0);
    this->declare_parameter("enable_nms", true);

    // Unknown region handling
    this->declare_parameter("treat_unknown_as_obstacle", true);

    // Performance parameters
    this->declare_parameter("use_bresenham_ray_casting", true);

    // Ray coverage parameters
    this->declare_parameter("min_coverage_ratio", 0.5);

    // Observation point filtering
    this->declare_parameter("score_threshold", 0.3);  // 默认降低到 0.3，增加观察点数量

    // Get parameters
    map_topic_ = this->get_parameter("map_topic").as_string();
    sample_step_ = this->get_parameter("sample_step").as_double();
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    ray_count_ = this->get_parameter("ray_count").as_int();
    update_rate_ = this->get_parameter("update_rate").as_double();

    // Get new parameters
    min_distance_threshold_ = this->get_parameter("min_distance_threshold").as_double();
    distance_weight_ = this->get_parameter("distance_weight").as_double();
    nms_buffer_radius_ = this->get_parameter("nms_buffer_radius").as_double();
    enable_nms_ = this->get_parameter("enable_nms").as_bool();
    treat_unknown_as_obstacle_ = this->get_parameter("treat_unknown_as_obstacle").as_bool();
    use_bresenham_ray_casting_ = this->get_parameter("use_bresenham_ray_casting").as_bool();
    min_coverage_ratio_ = this->get_parameter("min_coverage_ratio").as_double();
    score_threshold_ = this->get_parameter("score_threshold").as_double();

    RCLCPP_INFO(this->get_logger(), "Map Processor Node started with params:");
    RCLCPP_INFO(this->get_logger(), "  map_topic: %s", map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  sample_step: %.2f m", sample_step_);
    RCLCPP_INFO(this->get_logger(), "  robot_radius: %.2f m", robot_radius_);
    RCLCPP_INFO(this->get_logger(), "  ray_count: %d", ray_count_);
    RCLCPP_INFO(this->get_logger(), "  update_rate: %.2f Hz", update_rate_);
    RCLCPP_INFO(this->get_logger(), "Distance Transform:");
    RCLCPP_INFO(this->get_logger(), "  min_distance_threshold: %.2f m", min_distance_threshold_);
    RCLCPP_INFO(this->get_logger(), "  distance_weight: %.2f", distance_weight_);
    RCLCPP_INFO(this->get_logger(), "NMS:");
    RCLCPP_INFO(this->get_logger(), "  nms_buffer_radius: %.2f m", nms_buffer_radius_);
    RCLCPP_INFO(this->get_logger(), "  enable_nms: %s", enable_nms_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Unknown Region:");
    RCLCPP_INFO(this->get_logger(), "  treat_unknown_as_obstacle: %s", treat_unknown_as_obstacle_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Performance:");
    RCLCPP_INFO(this->get_logger(), "  use_bresenham_ray_casting: %s", use_bresenham_ray_casting_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  min_coverage_ratio: %.2f", min_coverage_ratio_);
    RCLCPP_INFO(this->get_logger(), "Observation Point Filtering:");
    RCLCPP_INFO(this->get_logger(), "  score_threshold: %.2f (points with score <= threshold are filtered out)", score_threshold_);

    // Create subscription and publisher
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(10).transient_local().reliable(),
        std::bind(&MapProcessorNode::mapCallback, this, std::placeholders::_1));

    observation_points_pub_ = this->create_publisher<rm_decision_interfaces::msg::ObservationPoints>(
        "/observation_points", 10);

    // Create timer for periodic updates
    auto timer_period = std::chrono::duration<double>(1.0 / update_rate_);
    timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&MapProcessorNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Waiting for map data on topic: %s", map_topic_.c_str());
}

void MapProcessorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received an empty map (0x0). Skipping processing...");
        return;
    }
    current_map_ = msg;
    RCLCPP_INFO(this->get_logger(), "Received map: %d x %d, resolution: %.3f m/pixel",
                msg->info.width, msg->info.height, msg->info.resolution);

    // Process and publish observation points immediately on first map receipt
    if (!first_map_received_) {
        first_map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First map received, processing observation points immediately...");
        processAndPublishObservationPoints();
    }
}

void MapProcessorNode::timerCallback()
{
    if (!current_map_) {
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Processing map for observation points...");
    processAndPublishObservationPoints();
}

void MapProcessorNode::processAndPublishObservationPoints()
{
    if (!current_map_) {
        return;
    }

    // Check if map has been updated
    if (!detectMapUpdate(current_map_)) {
        return;  // Skip processing if map hasn't changed
    }

    // Step 1: Binarize map (threshold > 50 for obstacles)
    processed_map_ = binarizeMap(*current_map_);

    // Step 2: Dilate map for robot radius
    processed_map_ = dilateMap(processed_map_, robot_radius_, current_map_->info.resolution);

    // Step 3: Compute distance transform
    distance_map_ = computeDistanceTransform(processed_map_);

    // Step 4: Sample grid points
    auto grid_points = sampleGridPoints(*current_map_, sample_step_);

    RCLCPP_INFO(this->get_logger(), "Sampled %zu candidate observation points", grid_points.size());

    // Step 5: Score each point with ray-casting and distance transform
    std::vector<double> scores;
    rm_decision_interfaces::msg::ObservationPoints obs_points;
    obs_points.header.stamp = this->now();
    obs_points.header.frame_id = current_map_->header.frame_id;

    // First pass: Score all points
    for (const auto & pt : grid_points) {
        double world_x, world_y;
        mapToWorld(current_map_->info, static_cast<int>(pt.x), static_cast<int>(pt.y),
                   world_x, world_y);

        double score = scoreObservationPoint(processed_map_, current_map_->info,
                                              static_cast<float>(world_x), static_cast<float>(world_y));
        scores.push_back(score);
    }

    // Apply NMS if enabled
    std::vector<size_t> keep_indices;
    if (enable_nms_) {
        keep_indices = applyNMS(grid_points, scores);
        RCLCPP_INFO(this->get_logger(), "NMS: %zu -> %zu points after suppression",
                    grid_points.size(), keep_indices.size());
    } else {
        // Keep all points
        keep_indices.resize(grid_points.size());
        for (size_t i = 0; i < grid_points.size(); i++) {
            keep_indices[i] = i;
        }
    }

    // Debug: 显示分数分布
    RCLCPP_INFO(this->get_logger(), "Score distribution (threshold=%.2f):", score_threshold_);
    for (size_t idx : keep_indices) {
        const auto & pt = grid_points[idx];
        double world_x, world_y;
        mapToWorld(current_map_->info, static_cast<int>(pt.x), static_cast<int>(pt.y), world_x, world_y);
        double score = scores[idx];
        const char* status = (score > score_threshold_) ? "[KEEP]" : "[FILTER]";
        RCLCPP_INFO(this->get_logger(), "  [%zu] (%.2f, %.2f) score=%.3f %s",
                    idx, world_x, world_y, score, status);
    }

    // Step 6: Filter and publish observation points
    uint32_t point_id = 0;
    for (size_t idx : keep_indices) {
        double score = scores[idx];

        // Only include points with reasonable visibility (using configurable threshold)
        if (score > score_threshold_) {
            const auto & pt = grid_points[idx];
            double world_x, world_y;
            mapToWorld(current_map_->info, static_cast<int>(pt.x), static_cast<int>(pt.y),
                       world_x, world_y);

            rm_decision_interfaces::msg::ObservationPoint obs_pt;
            obs_pt.point_id = point_id++;
            obs_pt.pose.position.x = world_x;
            obs_pt.pose.position.y = world_y;
            obs_pt.pose.position.z = 0.0;
            obs_pt.pose.orientation.w = 1.0;  // Default orientation
            obs_pt.score = static_cast<float>(score);

            obs_points.points.push_back(obs_pt);
        }
    }

    observation_points_pub_->publish(obs_points);
    RCLCPP_INFO(this->get_logger(), "Published %zu observation points",
                obs_points.points.size());
}

cv::Mat MapProcessorNode::binarizeMap(const nav_msgs::msg::OccupancyGrid & map)
{
    int width = map.info.width;
    int height = map.info.height;

    cv::Mat binary_map(height, width, CV_8UC1);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            int8_t value = map.data[idx];

            // Threshold > 50 for obstacles
            // Unknown (-1) handling based on treat_unknown_as_obstacle_ parameter
            if (value > 50) {
                binary_map.at<uint8_t>(y, x) = 255;  // Obstacle
            } else if (value == -1 && treat_unknown_as_obstacle_) {
                binary_map.at<uint8_t>(y, x) = 255;  // Unknown as Obstacle
            } else {
                binary_map.at<uint8_t>(y, x) = 0;    // Free space
            }
        }
    }

    return binary_map;
}

cv::Mat MapProcessorNode::dilateMap(const cv::Mat & binary_map, double robot_radius,
                                     double resolution)
{
    int kernel_size = static_cast<int>(robot_radius / resolution) * 2 + 1;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(kernel_size, kernel_size));

    cv::Mat dilated;
    cv::dilate(binary_map, dilated, kernel);

    return dilated;
}

std::vector<cv::Point2f> MapProcessorNode::sampleGridPoints(
    const nav_msgs::msg::OccupancyGrid & map, double step)
{
    std::vector<cv::Point2f> points;
    int step_pixels = static_cast<int>(step / map.info.resolution);

    for (int y = step_pixels / 2; y < static_cast<int>(map.info.height); y += step_pixels) {
        for (int x = step_pixels / 2; x < static_cast<int>(map.info.width); x += step_pixels) {
            // Only add points that are not in obstacles
            if (processed_map_.at<uint8_t>(y, x) == 0) {
                points.push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
            }
        }
    }

    return points;
}

double MapProcessorNode::scoreObservationPoint(const cv::Mat & map,
                                                const nav_msgs::msg::MapMetaData & info,
                                                float x, float y)
{
    double total_distance = 0.0;
    int valid_rays = 0;
    double max_range = 5.0;  // Maximum ray casting range in meters

    // Get grid coordinates for distance score
    int grid_x, grid_y;
    worldToMap(info, x, y, grid_x, grid_y);

    for (int i = 0; i < ray_count_; i++) {
        double angle = 2.0 * M_PI * i / ray_count_;
        double dist;

        // Choose ray casting algorithm based on parameter
        if (use_bresenham_ray_casting_) {
            dist = castRayBresenham(map, info, grid_x, grid_y, angle, max_range);
        } else {
            dist = castRayOnGrid(map, info, x, y, angle, max_range);
        }

        if (dist > 0) {
            total_distance += dist;
            valid_rays++;
        }
    }

    // Check coverage ratio
    double coverage_ratio = static_cast<double>(valid_rays) / ray_count_;
    if (coverage_ratio < min_coverage_ratio_) {
        return 0.0;  // Reject points with insufficient coverage
    }

    // Compute ray score (average normalized distance)
    double ray_score = (valid_rays > 0) ? (total_distance / valid_rays) / max_range : 0.0;

    // Compute distance score from distance transform
    double dist_score = getDistanceScore(grid_x, grid_y);

    // Combine scores with weights
    double combined_score = (1.0 - distance_weight_) * ray_score + distance_weight_ * dist_score;

    return combined_score;
}

double MapProcessorNode::castRayOnGrid(const cv::Mat & map,
                                        const nav_msgs::msg::MapMetaData & info,
                                        float start_x, float start_y,
                                        double angle, double max_range)
{
    double step_size = info.resolution * 0.5;
    double distance = 0.0;
    double dx = std::cos(angle) * step_size;
    double dy = std::sin(angle) * step_size;

    double current_x = start_x;
    double current_y = start_y;

    while (distance < max_range) {
        int grid_x, grid_y;
        if (!worldToMap(info, current_x, current_y, grid_x, grid_y)) {
            break;  // Out of map bounds
        }

        if (isObstacle(map, grid_x, grid_y)) {
            return distance;
        }

        current_x += dx;
        current_y += dy;
        distance += step_size;
    }

    return max_range;
}

double MapProcessorNode::castRayBresenham(const cv::Mat & map,
                                          const nav_msgs::msg::MapMetaData & info,
                                          int start_grid_x, int start_grid_y,
                                          double angle, double max_range)
{
    // Convert angle to direction vector
    double dx = std::cos(angle);
    double dy = std::sin(angle);

    // Calculate maximum grid steps
    int max_grid_steps = static_cast<int>(max_range / info.resolution) + 1;

    // Bresenham-like line algorithm
    // Using floating point for direction, but integer grid traversal
    int x = start_grid_x;
    int y = start_grid_y;

    // Determine step direction
    int step_x = (dx >= 0) ? 1 : -1;
    int step_y = (dy >= 0) ? 1 : -1;

    // Initialize decision variables
    double t_max_x = (dx == 0) ? max_grid_steps : std::abs(((step_x > 0) ? (x + 1) : x) - start_grid_x) / std::abs(dx);
    double t_max_y = (dy == 0) ? max_grid_steps : std::abs(((step_y > 0) ? (y + 1) : y) - start_grid_y) / std::abs(dy);

    double t_delta_x = (dx == 0) ? max_grid_steps : 1.0 / std::abs(dx);
    double t_delta_y = (dy == 0) ? max_grid_steps : 1.0 / std::abs(dy);

    int steps = 0;
    double distance = 0.0;

    while (steps < max_grid_steps) {
        // Check bounds
        if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
            // Calculate distance traveled
            distance = std::sqrt(std::pow(x - start_grid_x, 2) + std::pow(y - start_grid_y, 2)) * info.resolution;
            return distance;
        }

        // Check if current cell is obstacle
        if (isObstacle(map, x, y)) {
            distance = std::sqrt(std::pow(x - start_grid_x, 2) + std::pow(y - start_grid_y, 2)) * info.resolution;
            return distance;
        }

        // Move to next cell
        if (t_max_x < t_max_y) {
            x += step_x;
            distance = t_max_x * info.resolution;
            t_max_x += t_delta_x;
        } else {
            y += step_y;
            distance = t_max_y * info.resolution;
            t_max_y += t_delta_y;
        }

        steps++;
    }

    return max_range;
}

bool MapProcessorNode::isObstacle(const cv::Mat & map, int grid_x, int grid_y)
{
    if (grid_x < 0 || grid_x >= map.cols || grid_y < 0 || grid_y >= map.rows) {
        return true;  // Out of bounds treated as obstacle
    }
    return map.at<uint8_t>(grid_y, grid_x) > 0;
}

bool MapProcessorNode::worldToMap(const nav_msgs::msg::MapMetaData & info,
                                   double wx, double wy, int & mx, int & my)
{
    mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    my = static_cast<int>((wy - info.origin.position.y) / info.resolution);

    return (mx >= 0 && mx < static_cast<int>(info.width) &&
            my >= 0 && my < static_cast<int>(info.height));
}

void MapProcessorNode::mapToWorld(const nav_msgs::msg::MapMetaData & info,
                                   int mx, int my, double & wx, double & wy)
{
    wx = info.origin.position.x + (mx + 0.5) * info.resolution;
    wy = info.origin.position.y + (my + 0.5) * info.resolution;
}

bool MapProcessorNode::detectMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    if (!map) {
        return false;
    }

    // Check if map timestamp has changed
    if (map->header.stamp == last_map_timestamp_) {
        RCLCPP_DEBUG(this->get_logger(), "Map unchanged (timestamp: %d.%d)",
                     map->header.stamp.sec, map->header.stamp.nanosec);
        return false;
    }

    last_map_timestamp_ = map->header.stamp;
    RCLCPP_DEBUG(this->get_logger(), "Map updated (new timestamp: %d.%d)",
                 map->header.stamp.sec, map->header.stamp.nanosec);
    return true;
}

cv::Mat MapProcessorNode::computeDistanceTransform(const cv::Mat & binary_map)
{
    cv::Mat distance_map;
    cv::Mat binary_map_inv;

    // Invert binary map: obstacles (255) -> 0, free (0) -> 255
    // distanceTransform computes distance to nearest zero-valued pixel
    cv::bitwise_not(binary_map, binary_map_inv);

    // Compute distance transform (L2 distance)
    cv::distanceTransform(binary_map_inv, distance_map, cv::DIST_L2, 5);

    return distance_map;
}

double MapProcessorNode::getDistanceScore(int grid_x, int grid_y)
{
    if (distance_map_.empty()) {
        return 0.5;  // Default middle score if no distance map available
    }

    // Boundary check
    if (grid_x < 0 || grid_x >= distance_map_.cols ||
        grid_y < 0 || grid_y >= distance_map_.rows) {
        return 0.0;
    }

    // Get distance in pixels
    float distance_pixels = distance_map_.at<float>(grid_y, grid_x);

    // Convert to meters using current map resolution
    double distance_meters = distance_pixels * current_map_->info.resolution;

    // Apply minimum distance threshold
    if (distance_meters < min_distance_threshold_) {
        return 0.0;
    }

    // Normalize score: assume max meaningful distance is 3x the threshold
    double max_distance = min_distance_threshold_ * 3.0;
    double normalized_score = std::min(distance_meters / max_distance, 1.0);

    return normalized_score;
}

std::vector<size_t> MapProcessorNode::applyNMS(const std::vector<cv::Point2f>& points,
                                               const std::vector<double>& scores)
{
    if (points.empty() || points.size() != scores.size()) {
        return {};
    }

    std::vector<size_t> indices(scores.size());
    std::iota(indices.begin(), indices.end(), 0);

    // Sort indices by scores in descending order
    std::sort(indices.begin(), indices.end(),
              [&scores](size_t a, size_t b) { return scores[a] > scores[b]; });

    std::vector<size_t> keep_indices;
    std::vector<bool> suppressed(scores.size(), false);

    // Convert buffer radius to grid units
    double buffer_radius_pixels = nms_buffer_radius_ / current_map_->info.resolution;
    double buffer_radius_squared = buffer_radius_pixels * buffer_radius_pixels;

    for (size_t i : indices) {
        if (suppressed[i]) {
            continue;
        }

        keep_indices.push_back(i);

        // Suppress nearby points with lower scores
        const cv::Point2f& pt_i = points[i];
        for (size_t j : indices) {
            if (suppressed[j] || i == j) {
                continue;
            }

            const cv::Point2f& pt_j = points[j];
            double dx = pt_i.x - pt_j.x;
            double dy = pt_i.y - pt_j.y;
            double distance_squared = dx * dx + dy * dy;

            if (distance_squared < buffer_radius_squared) {
                suppressed[j] = true;
            }
        }
    }

    return keep_indices;
}

} // namespace rm_behavior_tree

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_behavior_tree::MapProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
