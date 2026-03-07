#include "icp_registration/icp_registration.hpp"
#include <Eigen/Geometry>
#include <limits>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/features/normal_3d.h>
#include <filesystem>

namespace icp {

IcpNode::IcpNode(const rclcpp::NodeOptions& options)
    : Node("icp_registration", options)
    , rough_iter_(10)
    , refine_iter_(5)
    , first_scan_(true)
{
    // 1. 初始化参数
    double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
    double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
    pcd_path_ = this->declare_parameter("prior_pcd_file", std::string(""));
    map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
    odom_frame_id_ = this->declare_parameter("odom_frame_id", std::string("odom"));
    robot_base_frame_ = this->declare_parameter("robot_base_frame", std::string("base_link"));
    laser_frame_id_ = this->declare_parameter("laser_frame_id", std::string("front_mid360"));
    pointcloud_topic_ = this->declare_parameter("pointcloud_topic", std::string("registered_scan"));
    xy_search_steps_ = this->declare_parameter("xy_search_steps", 3);

    thresh_ = this->declare_parameter("thresh", 0.15);
    xy_offset_ = this->declare_parameter("xy_offset", 1.0);
    yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
    yaw_resolution_ = this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
    
    // 初始位置参数 (x, y, z, roll, pitch, yaw)
    std::vector<double> init_pose_vec = this->declare_parameter("initial_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // 2. 初始化初始变换占位 (重要：解决 frame map does not exist)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_to_odom_.header.frame_id = map_frame_id_;
        map_to_odom_.child_frame_id = odom_frame_id_;
        map_to_odom_.transform.rotation.w = 1.0; 
        is_ready_ = true; // 允许发布 Identity 变换以维持 TF 树完整
    }

    // 3. 滤波器与点云初始化
    voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size, rough_leaf_size);
    voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size, refine_leaf_size);
    cloud_in_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // 4. 加载并处理地图
    loadMap();

    // 5. TF 监听与广播初始化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 6. 订阅者
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, qos,
        std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos,
        std::bind(&IcpNode::initialPoseCallback, this, std::placeholders::_1));

    // 7. 高频 TF 发布线程 (50Hz)
    tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
        rclcpp::Rate rate(50);
        while (rclcpp::ok()) {
            publishMapToOdom();
            rate.sleep();
        }
    });

    RCLCPP_INFO(this->get_logger(), "IcpNode initialized and map->odom broadcaster started.");
}

IcpNode::~IcpNode()
{
    if (tf_publisher_thread_ && tf_publisher_thread_->joinable()) {
        tf_publisher_thread_->join();
    }
}

void IcpNode::loadMap()
{
    if (!std::filesystem::exists(pcd_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Invalid PCD path: %s", pcd_path_.c_str());
        throw std::runtime_error("Invalid pcd path");
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile(pcd_path_, *raw_map) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file");
        return;
    }

    voxel_refine_filter_.setInputCloud(raw_map);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_refine(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_refine_filter_.filter(*filtered_refine);
    refine_map_ = addNorm(filtered_refine);

    voxel_rough_filter_.setInputCloud(filtered_refine);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_rough(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_rough_filter_.filter(*filtered_rough);
    rough_map_ = addNorm(filtered_rough);

    icp_rough_.setMaximumIterations(rough_iter_);
    icp_rough_.setInputTarget(rough_map_);
    icp_refine_.setMaximumIterations(refine_iter_);
    icp_refine_.setInputTarget(refine_map_);
}

void IcpNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pcl::fromROSMsg(*msg, *cloud_in_);
        last_scan_time_ = msg->header.stamp;
    }

    // 自动重定位：如果是第一帧点云，以坐标原点(或参数初值)自动触发一次匹配
    if (first_scan_) {
        RCLCPP_INFO(this->get_logger(), "First scan received. Auto-triggering registration...");
        auto pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        pose_msg->header = msg->header;
        // 默认设置为 (0,0,0) 或从参数获取的初始位姿
        pose_msg->pose.pose.orientation.w = 1.0; 
        initialPoseCallback(pose_msg);
        first_scan_ = false;
    }
}

/**
 * @brief 使用矩阵补偿逻辑计算 map->odom
 */
void IcpNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // 1. 获取目标机器人在地图中的位姿 (T_map_robot)
    Eigen::Isometry3d T_map_robot;
    tf2::fromMsg(msg->pose.pose, T_map_robot);

    // 2. 获取当前里程计对机器人的描述 (T_odom_robot)
    Eigen::Isometry3d T_odom_robot;
    try {
        // 使用 rclcpp::Time(0, 0, RCL_SYSTEM_TIME) 获取最新 TF
        auto tf_stamped = tf_buffer_->lookupTransform(
            odom_frame_id_, robot_base_frame_, tf2::TimePointZero);
        T_odom_robot = tf2::transformToEigen(tf_stamped);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    // 3. 准备执行 ICP 匹配以精化 T_map_lidar
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (cloud_in_->empty()) return;
        pcl::copyPointCloud(*cloud_in_, *current_cloud);
    }
    Eigen::Isometry3d T_map_odom_guess = T_map_robot * T_odom_robot.inverse();

    // 计算 T_map_lidar_guess = T_map_robot * T_robot_lidar
    // 这里为了简化，直接用 initialPose 作为匹配的初值点
    RCLCPP_INFO(this->get_logger(), "Starting Multi-Align ICP Refinement...");
    Eigen::Matrix4d T_map_lidar_refined = multiAlignSync(current_cloud, T_map_odom_guess.matrix());

    if (success_) {
        Eigen::Isometry3d T_map_odom;;
        T_map_odom.matrix() = T_map_lidar_refined;

        // 4. 更新用于发布的变量
        {
            std::lock_guard<std::mutex> lock(mutex_);
            map_to_odom_.transform = tf2::eigenToTransform(T_map_odom).transform;
            is_ready_ = true;
        }
        RCLCPP_INFO(this->get_logger(), "Relocalization Success. MSE: %f", score_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "ICP Refinement failed.");
    }
}

void IcpNode::publishMapToOdom()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_ready_) {
        // 关键优化：时间戳 + 0.1s 未来外推，补偿计算耗时，防止下游节点 Timed Out
        map_to_odom_.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
        tf_broadcaster_->sendTransform(map_to_odom_);
        //RCLCPP_INFO(this->get_logger(),"现在的 x, y 值: %f, %f , ", map_to_odom_.transform.translation.x, map_to_odom_.transform.translation.y);
    }
}

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d& init_guess)
{
    // --- 安全检查 ---
    success_ = false;
    score_ = std::numeric_limits<double>::infinity();

    if (!source || source->empty()) {
        return Eigen::Matrix4d::Zero();
    }

    // --- 更稳的 rotation->rpy（防 asin NaN）---
    static auto rotate2rpy = [](const Eigen::Matrix3d& rot) -> Eigen::Vector3d {
        // roll  = atan2(r32, r33)
        // pitch = asin(-r31)  (注意 clamp)
        // yaw   = atan2(r21, r11)
        const double roll = std::atan2(rot(2, 1), rot(2, 2));
        const double s = std::clamp(-rot(2, 0), -1.0, 1.0);
        const double pitch = std::asin(s);
        const double yaw = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    };

    // --- 从 init_guess 取位姿 ---
    Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);

    // 可选：纠正数值误差（让 rotation 更像正交阵）
    Eigen::Quaterniond q(rotation);
    q.normalize();
    rotation = q.toRotationMatrix();

    Eigen::Vector3d rpy = rotate2rpy(rotation);

    // --- 候选生成：xy 3x3 + yaw (按 yaw_offset_ / yaw_resolution_ 展开) ---
    int K = 0;
    if (yaw_resolution_ > 1e-9) {
        K = static_cast<int>(std::ceil(std::abs(yaw_offset_) / yaw_resolution_));
    }
    // 可选保险丝：避免 yaw_resolution_ 太小导致候选爆炸
    K = std::min(K, 36);

    std::vector<Eigen::Matrix4f> candidates;
    candidates.reserve(static_cast<size_t>(9 * (2 * K + 1)));

    const Eigen::AngleAxisf rollAngle(static_cast<float>(rpy(0)), Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitchAngle(static_cast<float>(rpy(1)), Eigen::Vector3f::UnitY());

    int N = std::max(0, xy_search_steps_);
    N = std::min(N, 50);  // 保险丝：避免参数写错候选爆炸
    for (int i = -N; i <= N; ++i) {
        for (int j = -N; j <= N; ++j) {
            for (int k = -K; k <= K; ++k) {
                Eigen::Vector3f pos(static_cast<float>(xyz(0) + i * xy_offset_),
                                    static_cast<float>(xyz(1) + j * xy_offset_),
                                    static_cast<float>(xyz(2)));

                const float yaw = static_cast<float>(rpy(2) + k * yaw_resolution_);
                const Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

                Eigen::Matrix4f temp_pose = Eigen::Matrix4f::Identity();
                temp_pose.block<3, 3>(0, 0) = (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
                temp_pose.block<3, 1>(0, 3) = pos;
                candidates.push_back(temp_pose);
            }
        }
    }

    // --- 粗匹配：降采样 + 法向量 ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_rough_filter_.setInputCloud(source);
    voxel_rough_filter_.filter(*rough_source);
    if (rough_source->empty()) {
        return Eigen::Matrix4d::Zero();
    }

    PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
    if (!rough_source_norm || rough_source_norm->empty()) {
        return Eigen::Matrix4d::Zero();
    }

    Eigen::Matrix4f best_rough_transform = Eigen::Matrix4f::Identity();
    double best_rough_score = std::numeric_limits<double>::infinity();
    bool rough_converge = false;

    // 计时用 steady_clock（更适合测耗时）
    const auto tic = std::chrono::steady_clock::now();

    icp_rough_.setInputSource(rough_source_norm);

    for (const auto& init_pose : candidates) {
        PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);
        icp_rough_.align(*align_point, init_pose);

        if (!icp_rough_.hasConverged()) {
            continue;
        }

        const double s = icp_rough_.getFitnessScore();
        if (s < best_rough_score) {
            best_rough_score = s;
            best_rough_transform = icp_rough_.getFinalTransformation();
            rough_converge = true;
        }
    }

    if (!rough_converge) {
        return Eigen::Matrix4d::Zero();
    }

    // --- 精匹配：更细降采样 + 法向量 ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_refine_filter_.setInputCloud(source);
    voxel_refine_filter_.filter(*refine_source);
    if (refine_source->empty()) {
        return Eigen::Matrix4d::Zero();
    }

    PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
    if (!refine_source_norm || refine_source_norm->empty()) {
        return Eigen::Matrix4d::Zero();
    }

    PointCloudXYZIN::Ptr final_align_point(new PointCloudXYZIN);

    icp_refine_.setInputSource(refine_source_norm);
    icp_refine_.align(*final_align_point, best_rough_transform);

    if (!icp_refine_.hasConverged()) {
        return Eigen::Matrix4d::Zero();
    }

    score_ = icp_refine_.getFitnessScore();
    if (score_ < thresh_) {
        success_ = true;

        const auto toc = std::chrono::steady_clock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count();

        RCLCPP_INFO(this->get_logger(), "ICP Align used: %ld ms, Score: %f (rough best: %f)",
                    static_cast<long>(ms), score_, best_rough_score);

        return icp_refine_.getFinalTransformation().cast<double>();
    }

    return Eigen::Matrix4d::Zero();
}

PointCloudXYZIN::Ptr IcpNode::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZI>);
    searchTree->setInputCloud(cloud);

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setSearchMethod(searchTree);
    ne.setKSearch(15);
    ne.compute(*normals);

    PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
    pcl::concatenateFields(*cloud, *normals, *out);
    return out;
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)