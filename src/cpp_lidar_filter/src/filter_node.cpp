#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <mutex>

class LidarRangeSplitFilterNode : public rclcpp::Node
{
public:
  LidarRangeSplitFilterNode()
  : Node("lidar_filter_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("input_topic", "/livox/lidar");
    this->declare_parameter<std::string>("odom_topic", "/lidar_odometry");

    this->declare_parameter<std::string>(
      "near_output_topic", "/livox/lidar_filtered_near");
    this->declare_parameter<std::string>(
      "far_output_topic", "/livox/lidar_filtered_far");

    // 点云高度判断使用的目标坐标系。
    // local_costmap 是 odom，所以默认 odom。
    this->declare_parameter<std::string>("target_frame", "odom");

    this->declare_parameter<bool>("enable_filter", true);
    this->declare_parameter<bool>("enable_z_compensation", true);
    this->declare_parameter<bool>("enable_ground_filter", true);

    // true：没收到 odom 时直接跳过点云。
    // false：没收到 odom 时 dz = 0，继续处理。
    this->declare_parameter<bool>("require_odom", true);

    // 是否重新给输出点云打时间戳。
    // 建议 true。
    // 输出 stamp 会使用 lookupTransform(Time(0)) 返回的 TF 时间戳。
    this->declare_parameter<bool>("restamp_output_cloud", true);

    // 去车体附近点云。
    this->declare_parameter<double>("self_filter_radius", 0.55);

    // near: 近距离慢衰减层。
    this->declare_parameter<double>("near_min_radius", 0.55);
    this->declare_parameter<double>("near_max_radius", 1.0);

    // far: 远距离快衰减层。
    this->declare_parameter<double>("far_min_radius", 1.0);
    this->declare_parameter<double>("far_max_radius", 9.0);

    // odom 高度补偿后的地面过滤阈值。
    this->declare_parameter<double>("ground_clearance", 0.12);
    this->declare_parameter<double>("max_obstacle_height", 1.5);

    // lookupTransform 等待时间。
    this->declare_parameter<double>("tf_timeout", 0.1);

    this->declare_parameter<bool>("debug_log", false);

    input_topic_ = this->get_parameter("input_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();

    near_output_topic_ = this->get_parameter("near_output_topic").as_string();
    far_output_topic_ = this->get_parameter("far_output_topic").as_string();

    target_frame_ = this->get_parameter("target_frame").as_string();

    enable_filter_ = this->get_parameter("enable_filter").as_bool();
    enable_z_compensation_ = this->get_parameter("enable_z_compensation").as_bool();
    enable_ground_filter_ = this->get_parameter("enable_ground_filter").as_bool();
    require_odom_ = this->get_parameter("require_odom").as_bool();
    restamp_output_cloud_ = this->get_parameter("restamp_output_cloud").as_bool();

    self_filter_radius_ = this->get_parameter("self_filter_radius").as_double();

    near_min_radius_ = this->get_parameter("near_min_radius").as_double();
    near_max_radius_ = this->get_parameter("near_max_radius").as_double();

    far_min_radius_ = this->get_parameter("far_min_radius").as_double();
    far_max_radius_ = this->get_parameter("far_max_radius").as_double();

    ground_clearance_ = this->get_parameter("ground_clearance").as_double();
    max_obstacle_height_ = this->get_parameter("max_obstacle_height").as_double();

    tf_timeout_ = this->get_parameter("tf_timeout").as_double();

    debug_log_ = this->get_parameter("debug_log").as_bool();

    self_filter_radius_sq_ =
      static_cast<float>(self_filter_radius_ * self_filter_radius_);

    near_min_sq_ =
      static_cast<float>(near_min_radius_ * near_min_radius_);
    near_max_sq_ =
      static_cast<float>(near_max_radius_ * near_max_radius_);

    far_min_sq_ =
      static_cast<float>(far_min_radius_ * far_min_radius_);
    far_max_sq_ =
      static_cast<float>(far_max_radius_ * far_max_radius_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarRangeSplitFilterNode::odomCallback, this, std::placeholders::_1));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LidarRangeSplitFilterNode::cloudCallback, this, std::placeholders::_1));

    near_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      near_output_topic_,
      rclcpp::SensorDataQoS());

    far_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      far_output_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Input cloud topic: %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());

    RCLCPP_INFO(this->get_logger(), "Near output: %s", near_output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Far output: %s", far_output_topic_.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "self_filter=%.2f, near=%.2f~%.2f, far=%.2f~%.2f, ground_clearance=%.2f, max_h=%.2f",
      self_filter_radius_,
      near_min_radius_, near_max_radius_,
      far_min_radius_, far_max_radius_,
      ground_clearance_, max_obstacle_height_);

    RCLCPP_INFO(
      this->get_logger(),
      "restamp_output_cloud=%s, tf_timeout=%.3f",
      restamp_output_cloud_ ? "true" : "false",
      tf_timeout_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    current_odom_z_ = msg->pose.pose.position.z;
    has_odom_ = true;

    // 第一次收到 odom 时，记录参考高度。
    //
    // 后续补偿：
    // dz = current_odom_z - odom_z_ref
    //
    // 这样减的是“相对初始平地的高度变化量”，
    // 不是无脑减 current_odom_z。
    if (!has_odom_z_ref_) {
      odom_z_ref_ = current_odom_z_;
      has_odom_z_ref_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Set odom_z_ref = %.3f",
        odom_z_ref_);
    }
  }

  bool getOdomZInfo(double & current_z, double & ref_z)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);

    if (!has_odom_ || !has_odom_z_ref_) {
      return false;
    }

    current_z = current_odom_z_;
    ref_z = odom_z_ref_;
    return true;
  }

  void initOutputCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    sensor_msgs::msg::PointCloud2 & out,
    const rclcpp::Time & output_stamp)
  {
    // 输出仍然保持输入点云 frame_id。
    // 通常是 front_mid360。
    //
    // 这样 STVL 后续 raytrace / clearing 时，
    // 不会把 odom 原点误认为传感器原点。
    out.header = msg->header;

    // 关键：
    // 输出点云的 stamp 不用原始 Livox stamp，也不直接用 now()。
    // 而是使用 lookupTransform(Time(0)) 返回的 TF 时间戳。
    //
    // 这样 STVL / costmap 的 MessageFilter 能在 TF cache 中找到对应时间。
    if (restamp_output_cloud_) {
      out.header.stamp = output_stamp;
    }

    out.fields = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step = msg->point_step;
    out.is_dense = false;

    out.height = 1;
    out.width = 0;
    out.row_step = 0;

    // 一次性分配最大可能空间，避免循环中频繁扩容。
    out.data.resize(msg->data.size());
  }

  void finalizeOutputCloud(
    sensor_msgs::msg::PointCloud2 & out,
    size_t write_offset,
    size_t kept)
  {
    out.data.resize(write_offset);
    out.height = 1;
    out.width = static_cast<uint32_t>(kept);
    out.row_step = out.width * out.point_step;
  }

  bool lookupTransforms(
    const std::string & cloud_frame,
    tf2::Transform & tf_target_from_cloud,
    tf2::Transform & tf_cloud_from_target,
    rclcpp::Time & output_stamp)
  {
    try {
      // Time(0) 表示查询最新可用 TF。
      //
      // 避免用原始点云 stamp 时出现：
      // Lookup would require extrapolation into the future
      const rclcpp::Time latest_tf_time(0, 0, this->get_clock()->get_clock_type());

      // 只查一次 TF：
      //
      // target_frame_ <- cloud_frame
      //
      // 例如：
      // odom <- front_mid360
      //
      // 这个变换用于把 front_mid360 下的点变换到 odom 下。
      geometry_msgs::msg::TransformStamped tf_msg =
        tf_buffer_.lookupTransform(
          target_frame_,
          cloud_frame,
          latest_tf_time,
          rclcpp::Duration::from_seconds(tf_timeout_));

      tf2::fromMsg(tf_msg.transform, tf_target_from_cloud);

      // 反向变换不再单独查 TF。
      // 直接由正向 TF 求逆：
      //
      // 如果正向是 odom <- front_mid360，
      // inverse 后就是 front_mid360 <- odom。
      tf_cloud_from_target = tf_target_from_cloud.inverse();

      // 关键：
      // 输出点云 stamp 使用这条 TF 的真实时间戳。
      //
      // 不用 now()，避免输出点云比 TF 最新数据还新。
      // 不用原始点云 stamp，避免输出点云比 TF cache 最早数据还旧。
      output_stamp = rclcpp::Time(
        tf_msg.header.stamp,
        this->get_clock()->get_clock_type());

      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 当前版本只支持 XYZI：
    //
    // x float32: offset 0
    // y float32: offset 4
    // z float32: offset 8
    // intensity float32: offset 12
    //
    // point_step = 16
    if (msg->point_step != 16) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Unsupported PointCloud2 format: point_step = %u, expected 16",
        msg->point_step);
      return;
    }

    const size_t point_count =
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

    if (point_count == 0) {
      return;
    }

    // ===============================
    // 1. 获取 odom.z
    // ===============================
    double odom_z_now = 0.0;
    double odom_z_ref = 0.0;

    const bool odom_ok = getOdomZInfo(odom_z_now, odom_z_ref);

    if (!odom_ok && require_odom_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No odom received yet, skip cloud");
      return;
    }

    // dz 是机器人相对初始平地高度的变化量。
    //
    // 平地：dz ≈ 0
    // 上中央高地：dz > 0
    // 下坡/低地：dz < 0
    const double dz = odom_ok ? (odom_z_now - odom_z_ref) : 0.0;

    // ===============================
    // 2. 查询 TF
    // ===============================
    const std::string cloud_frame = msg->header.frame_id;

    if (cloud_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Input cloud frame_id is empty");
      return;
    }

    tf2::Transform tf_target_from_cloud;
    tf2::Transform tf_cloud_from_target;

    rclcpp::Time output_stamp(0, 0, this->get_clock()->get_clock_type());

    if (!lookupTransforms(
        cloud_frame,
        tf_target_from_cloud,
        tf_cloud_from_target,
        output_stamp))
    {
      return;
    }

    // ===============================
    // 3. 初始化输出点云
    // ===============================
    sensor_msgs::msg::PointCloud2 near_cloud;
    sensor_msgs::msg::PointCloud2 far_cloud;

    initOutputCloud(msg, near_cloud, output_stamp);
    initOutputCloud(msg, far_cloud, output_stamp);

    const uint8_t * src = msg->data.data();

    uint8_t * near_dst = near_cloud.data.data();
    uint8_t * far_dst = far_cloud.data.data();

    size_t near_write_offset = 0;
    size_t far_write_offset = 0;

    size_t near_kept = 0;
    size_t far_kept = 0;

    size_t invalid_count = 0;
    size_t self_filtered_count = 0;
    size_t too_far_count = 0;
    size_t ground_filtered_count = 0;
    size_t high_filtered_count = 0;

    // ===============================
    // 4. 不过滤模式，仅用于调试
    // ===============================
    if (!enable_filter_) {
      std::memcpy(near_dst, src, msg->data.size());
      std::memcpy(far_dst, src, msg->data.size());

      near_write_offset = msg->data.size();
      far_write_offset = msg->data.size();

      near_kept = point_count;
      far_kept = point_count;

      finalizeOutputCloud(near_cloud, near_write_offset, near_kept);
      finalizeOutputCloud(far_cloud, far_write_offset, far_kept);

      near_pub_->publish(near_cloud);
      far_pub_->publish(far_cloud);
      return;
    }

    // ===============================
    // 5. 主循环
    // ===============================
    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t * p = src + i * 16;

      float x_lidar = 0.0f;
      float y_lidar = 0.0f;
      float z_lidar = 0.0f;

      std::memcpy(&x_lidar, p + 0, sizeof(float));
      std::memcpy(&y_lidar, p + 4, sizeof(float));
      std::memcpy(&z_lidar, p + 8, sizeof(float));

      if (
        !std::isfinite(x_lidar) ||
        !std::isfinite(y_lidar) ||
        !std::isfinite(z_lidar))
      {
        invalid_count++;
        continue;
      }

      // 5.1 front_mid360 原始坐标系下半径初筛。
      //
      // 使用 x/y 平面距离，不使用 3D 距离。
      //
      // 作用：
      // 1. r < self_filter_radius：去掉车体附近点云。
      // 2. r > far_max_radius：去掉过远点。
      // 3. 减少后续 TF 计算量。
      const float r2_lidar = x_lidar * x_lidar + y_lidar * y_lidar;

      if (r2_lidar < self_filter_radius_sq_) {
        self_filtered_count++;
        continue;
      }

      if (r2_lidar >= far_max_sq_) {
        too_far_count++;
        continue;
      }

      // 5.2 点从 front_mid360 转到 odom / target_frame。
      tf2::Vector3 point_cloud_frame(
        static_cast<double>(x_lidar),
        static_cast<double>(y_lidar),
        static_cast<double>(z_lidar));

      tf2::Vector3 point_target_frame =
        tf_target_from_cloud * point_cloud_frame;

      // 5.3 在 odom / target_frame 下做高度补偿。
      //
      // 注意：
      // z_comp = point_target_frame.z() - (odom_z_now - odom_z_ref)
      //
      // 不是：
      // z_comp = point_target_frame.z() - odom_z_now
      double z_comp = point_target_frame.z();

      if (enable_z_compensation_) {
        z_comp = z_comp - dz;
      }

      // 5.4 地面过滤。
      //
      // z_comp 太低：认为是地面或地面噪声。
      // z_comp 太高：认为不影响底盘通行。
      if (enable_ground_filter_) {
        if (z_comp < ground_clearance_) {
          ground_filtered_count++;
          continue;
        }

        if (z_comp > max_obstacle_height_) {
          high_filtered_count++;
          continue;
        }
      }

      // 5.5 把补偿后的 odom 点转回原始 cloud_frame。
      //
      // 输出点云仍然是 front_mid360 frame。
      // 这样 STVL 后面仍然可以根据点云 frame 找传感器 TF。
      point_target_frame.setZ(z_comp);

      tf2::Vector3 point_cloud_frame_out =
        tf_cloud_from_target * point_target_frame;

      const float x_out = static_cast<float>(point_cloud_frame_out.x());
      const float y_out = static_cast<float>(point_cloud_frame_out.y());
      const float z_out = static_cast<float>(point_cloud_frame_out.z());

      if (
        !std::isfinite(x_out) ||
        !std::isfinite(y_out) ||
        !std::isfinite(z_out))
      {
        invalid_count++;
        continue;
      }

      // 5.6 near / far 分层输出。
      //
      // 分层仍然使用原始 front_mid360 下的 r2_lidar。
      // 因为 near/far 表达的是雷达到点的局部水平距离。
      if (r2_lidar >= near_min_sq_ && r2_lidar < near_max_sq_) {
        uint8_t * out_p = near_dst + near_write_offset;

        // 先复制原始点全部字段，例如 intensity。
        std::memcpy(out_p, p, 16);

        // 再覆盖 x/y/z 为补偿后的坐标。
        std::memcpy(out_p + 0, &x_out, sizeof(float));
        std::memcpy(out_p + 4, &y_out, sizeof(float));
        std::memcpy(out_p + 8, &z_out, sizeof(float));

        near_write_offset += 16;
        near_kept++;
      } else if (r2_lidar >= far_min_sq_ && r2_lidar < far_max_sq_) {
        uint8_t * out_p = far_dst + far_write_offset;

        std::memcpy(out_p, p, 16);

        std::memcpy(out_p + 0, &x_out, sizeof(float));
        std::memcpy(out_p + 4, &y_out, sizeof(float));
        std::memcpy(out_p + 8, &z_out, sizeof(float));

        far_write_offset += 16;
        far_kept++;
      }
    }

    // ===============================
    // 6. 收尾并发布
    // ===============================
    finalizeOutputCloud(near_cloud, near_write_offset, near_kept);
    finalizeOutputCloud(far_cloud, far_write_offset, far_kept);

    near_pub_->publish(near_cloud);
    far_pub_->publish(far_cloud);

    if (debug_log_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "input=%zu near=%zu far=%zu invalid=%zu self=%zu too_far=%zu ground=%zu high=%zu dz=%.3f out_stamp=%.3f",
        point_count,
        near_kept,
        far_kept,
        invalid_count,
        self_filtered_count,
        too_far_count,
        ground_filtered_count,
        high_filtered_count,
        dz,
        output_stamp.seconds());
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr near_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr far_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string input_topic_;
  std::string odom_topic_;
  std::string near_output_topic_;
  std::string far_output_topic_;
  std::string target_frame_;

  bool enable_filter_{true};
  bool enable_z_compensation_{true};
  bool enable_ground_filter_{true};
  bool require_odom_{true};
  bool restamp_output_cloud_{true};
  bool debug_log_{false};

  double self_filter_radius_{0.55};

  double near_min_radius_{0.55};
  double near_max_radius_{1.0};

  double far_min_radius_{1.0};
  double far_max_radius_{9.0};

  double ground_clearance_{0.12};
  double max_obstacle_height_{1.5};

  double tf_timeout_{0.1};

  float self_filter_radius_sq_{0.55f * 0.55f};

  float near_min_sq_{0.55f * 0.55f};
  float near_max_sq_{1.0f * 1.0f};

  float far_min_sq_{1.0f * 1.0f};
  float far_max_sq_{9.0f * 9.0f};

  std::mutex odom_mutex_;

  bool has_odom_{false};
  bool has_odom_z_ref_{false};

  double current_odom_z_{0.0};
  double odom_z_ref_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRangeSplitFilterNode>());
  rclcpp::shutdown();
  return 0;
}