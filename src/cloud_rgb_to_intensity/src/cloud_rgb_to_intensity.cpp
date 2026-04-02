/**
 * @file cloud_rgb_to_intensity.cpp
 * @brief 高性能 PointXYZRGB 转 PointXYZI 节点 (修复版)
 * * 适配 terrain_analysis，解决 rclcpp::Time 与 msg::Time 比较报错问题。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>

class CloudRgbToIntensity : public rclcpp::Node {
public:
    CloudRgbToIntensity()
        : Node("cloud_rgb_to_intensity")
    {
        // 参数声明
        this->declare_parameter("input_topic", "odin1/cloud_slam");
        this->declare_parameter("output_topic", "registered_scan");

        std::string input_topic = this->get_parameter("input_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        // QoS 配置：传感器数据通常使用 BestEffort
        // 【核心修复：匹配 RELIABLE 属性】
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                       .reliable()
                       .durability_volatile();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, qos,
            std::bind(&CloudRgbToIntensity::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

        RCLCPP_INFO(this->get_logger(), "Node started: %s -> %s", input_topic.c_str(), output_topic.c_str());
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. 构造输出消息
        auto out_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        out_msg->header.stamp = msg->header.stamp;
        out_msg->header.frame_id = "odin1";  // 手动添加 frame 为 odin1
        out_msg->height = msg->height;
        out_msg->width = msg->width;
        out_msg->is_dense = msg->is_dense;
        out_msg->is_bigendian = msg->is_bigendian;

        // --- 【核心修复：手动定义字段，确保 intensity 100% 存在】 ---
        sensor_msgs::PointCloud2Modifier modifier(*out_msg);
        out_msg->fields.clear(); // 彻底清空，防止干扰
        // 参数依次为：字段数, 名字, 数量, 数据类型, 名字, 数量...
        modifier.setPointCloud2Fields(4, 
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        modifier.resize(msg->width * msg->height);

        // 4. 高效数据填充
        sensor_msgs::PointCloud2ConstIterator<float> it_in_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_in_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_in_z(*msg, "z");

        sensor_msgs::PointCloud2Iterator<float> it_out_x(*out_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> it_out_y(*out_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> it_out_z(*out_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> it_out_i(*out_msg, "intensity");

        for (; it_in_x != it_in_x.end();
            ++it_in_x, ++it_in_y, ++it_in_z,
            ++it_out_x, ++it_out_y, ++it_out_z, ++it_out_i) {
            *it_out_x = *it_in_x;
            *it_out_y = *it_in_y;
            *it_out_z = *it_in_z;
            *it_out_i = 0.0f; // 初始强度为0，terrain_analysis会根据时间戳重新计算
        }

        

        pub_->publish(std::move(out_msg));

    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudRgbToIntensity>());
    rclcpp::shutdown();
    return 0;
}