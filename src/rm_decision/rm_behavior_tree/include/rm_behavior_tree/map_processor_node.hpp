#ifndef RM_BEHAVIOR_TREE__MAP_PROCESSOR_NODE_HPP_
#define RM_BEHAVIOR_TREE__MAP_PROCESSOR_NODE_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rm_decision_interfaces/msg/observation_points.hpp>
#include <rm_decision_interfaces/msg/observation_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace rm_behavior_tree
{

class MapProcessorNode : public rclcpp::Node
{
public:
    MapProcessorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // ROS2 interface
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<rm_decision_interfaces::msg::ObservationPoints>::SharedPtr observation_points_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string map_topic_;
    double sample_step_;
    double robot_radius_;
    int ray_count_;
    double update_rate_;

    // Distance transform parameters
    double min_distance_threshold_;
    double distance_weight_;

    // NMS parameters
    double nms_buffer_radius_;
    bool enable_nms_;

    // Unknown region handling
    bool treat_unknown_as_obstacle_;

    // Performance parameters
    bool use_bresenham_ray_casting_;

    // Ray coverage parameters
    double min_coverage_ratio_;

    // Observation point filtering
    double score_threshold_;  // Minimum score for an observation point to be published

    // Map data
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    cv::Mat processed_map_;
    cv::Mat distance_map_;
    bool first_map_received_;

    // Map change detection
    rclcpp::Time last_map_timestamp_;

    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void timerCallback();

    // Main processing function
    void processAndPublishObservationPoints();

    // Map processing functions
    cv::Mat binarizeMap(const nav_msgs::msg::OccupancyGrid & map);
    cv::Mat dilateMap(const cv::Mat & binary_map, double robot_radius, double resolution);
    std::vector<cv::Point2f> sampleGridPoints(const nav_msgs::msg::OccupancyGrid & map, double step);
    double scoreObservationPoint(const cv::Mat & map, const nav_msgs::msg::MapMetaData & info, float x, float y);
    double castRayOnGrid(const cv::Mat & map, const nav_msgs::msg::MapMetaData & info,
                         float start_x, float start_y, double angle, double max_range);
    double castRayBresenham(const cv::Mat & map, const nav_msgs::msg::MapMetaData & info,
                            int start_grid_x, int start_grid_y, double angle, double max_range);
    bool isObstacle(const cv::Mat & map, int grid_x, int grid_y);

    // Distance transform functions
    cv::Mat computeDistanceTransform(const cv::Mat & binary_map);
    double getDistanceScore(int grid_x, int grid_y);

    // NMS function
    std::vector<size_t> applyNMS(const std::vector<cv::Point2f>& points,
                                 const std::vector<double>& scores);

    // Map change detection
    bool detectMapUpdate(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

    // Utility functions
    bool worldToMap(const nav_msgs::msg::MapMetaData & info, double wx, double wy,
                    int & mx, int & my);
    void mapToWorld(const nav_msgs::msg::MapMetaData & info, int mx, int my,
                    double & wx, double & wy);
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__MAP_PROCESSOR_NODE_HPP_
