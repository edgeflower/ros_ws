#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <fstream>
#include <filesystem>

#include "polygon_manager/polygon_manager.hpp"

namespace polygon_manager {

/**
 * @class PolygonManagerNode
 * @brief ROS2 节点，连接 PolygonManager 和 ROS 通信
 * 
 * 节点功能：
 * ========
 * 1. 订阅 RViz 点击事件 (/clicked_point)
 * 2. 订阅敌方位置信息 (/enemy_position)
 * 3. 订阅地图信息 (/map)
 * 4. 发布可视化 Marker (/polygon_areas/visualization_marker_array)
 * 5. 发布敌方检测结果 (/enemy_area_state)
 * 6. 提供交互式 polygon 创建
 * 
 * 地图支持：
 * ========
 * - slam_toolbox: /map topic, nav_msgs/msg/OccupancyGrid
 * - nav2_map_server: /map 静态地图，map.yaml + pgm
 * 都统一使用 "map" 坐标系
 * 
 * 交互流程：
 * ========
 * 1. 使用 RViz 的 "Publish Point" 工具
 * 2. 在地图上依次点击多边形顶点
 * 3. 最后调用 close 命令完成多边形
 * 4. 系统自动可视化并保存配置
 */
class PolygonManagerNode : public rclcpp::Node {
 public:
  /**
   * @brief 构造节点
   */
  PolygonManagerNode() : Node("polygon_manager_node") {
    RCLCPP_INFO(this->get_logger(),
                "RoboMaster Polygon Manager Node Starting...");

    // 初始化管理器
    manager_ = std::make_unique<PolygonManager>();

    // 声明参数
    this->declare_parameter("config_file", "config/areas.yaml");
    this->declare_parameter("update_frequency", 10.0);
    this->declare_parameter("map_file", "");

    std::string config_file = this->get_parameter("config_file").as_string();
    double update_freq = this->get_parameter("update_frequency").as_double();

    // 尝试加载配置文件
    // 首先尝试从参数指定的路径加载
    if (loadConfigFile(config_file)) {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully loaded config from: %s", config_file.c_str());
    }

    // 创建离线地图发布者（transient_local QoS 保证后加入的节点也能收到）
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.transient_local();
    map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos);

    // 尝试加载离线地图
    std::string map_file = this->get_parameter("map_file").as_string();
    if (!map_file.empty()) {
      loadOfflineMap(map_file);
    }

    // 创建发布者
    marker_publisher_ = this->create_publisher<
        visualization_msgs::msg::MarkerArray>(
            "/polygon_areas/visualization_marker_array", 10);

    enemy_state_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/enemy_area_state", 10);

    // 创建订阅者
    clicked_point_subscription_ = this->create_subscription<
        geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10,
            std::bind(&PolygonManagerNode::onClickedPoint, this,
                      std::placeholders::_1));

    enemy_position_subscription_ = this->create_subscription<
        geometry_msgs::msg::PoseStamped>(
            "/enemy_position", 10,
            std::bind(&PolygonManagerNode::onEnemyPosition, this,
                      std::placeholders::_1));

    map_subscription_ = this->create_subscription<
        nav_msgs::msg::OccupancyGrid>(
            "/map", 10,
            std::bind(&PolygonManagerNode::onMapReceived, this,
                      std::placeholders::_1));

    // 创建定时器，周期性发布 Marker
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / update_freq)),
        std::bind(&PolygonManagerNode::publishMarkers, this));

    // 区域类型选择订阅
    area_type_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/polygon_manager/set_area_type", 10,
        std::bind(&PolygonManagerNode::onSetAreaType, this,
                  std::placeholders::_1));

    // 闭合多边形服务
    close_polygon_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/polygon_manager/close_polygon",
        std::bind(&PolygonManagerNode::onClosePolygon, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 取消多边形服务
    cancel_polygon_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/polygon_manager/cancel_polygon",
        std::bind(&PolygonManagerNode::onCancelPolygon, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 状态发布
    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/polygon_manager/status", 10);
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PolygonManagerNode::publishStatus, this));

    RCLCPP_INFO(this->get_logger(), "Polygon Manager Node Ready!");
    RCLCPP_INFO(this->get_logger(),
                "Commands:");
    RCLCPP_INFO(this->get_logger(),
                "  Set type: ros2 topic pub /polygon_manager/set_area_type "
                "std_msgs/msg/String '{data: \"forbidden\"}' --once");
    RCLCPP_INFO(this->get_logger(),
                "  Close polygon: ros2 service call /polygon_manager/close_polygon std_srvs/srv/Trigger");
    RCLCPP_INFO(this->get_logger(),
                "  Cancel polygon: ros2 service call /polygon_manager/cancel_polygon std_srvs/srv/Trigger");
    RCLCPP_INFO(this->get_logger(),
                "  Area types: forbidden, protect, patrol, attack");
  }

 private:
  std::unique_ptr<PolygonManager> manager_;

  // 发布者
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      enemy_state_publisher_;

  // 订阅者
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      clicked_point_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      enemy_position_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      map_subscription_;

  // 离线地图发布者
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

  // 离线地图定时发布器（周期性重发，确保 RViz 能收到）
  rclcpp::TimerBase::SharedPtr map_publish_timer_;

  // 缓存的离线地图
  nav_msgs::msg::OccupancyGrid cached_map_;
  bool has_offline_map_ = false;

  // 定时器
  rclcpp::TimerBase::SharedPtr update_timer_;

  // 区域类型编辑
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr area_type_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_polygon_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_polygon_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  AreaType current_drawing_type_ = AreaType::FORBIDDEN_AREA;

  // 当前敌方位置（缓存最后接收到的敌方位置）
  geometry_msgs::msg::PoseStamped last_enemy_pose_;
  bool has_enemy_position_ = false;

  /**
   * @brief 加载配置文件
   * 
   * 尝试从多个位置加载配置文件
   */
  bool loadConfigFile(const std::string& config_file) {
    // 尝试相对路径
    if (manager_->loadFromYaml(config_file)) {
      return true;
    }

    // 尝试从包共享目录加载
    try {
      std::string package_dir = ament_index_cpp::get_package_share_directory(
          "polygon_manager");
      std::string full_path = package_dir + "/" + config_file;
      if (manager_->loadFromYaml(full_path)) {
        return true;
      }
    } catch (const std::exception& e) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Could not get package share directory: %s", e.what());
    }

    RCLCPP_WARN(this->get_logger(),
                "Could not load config file: %s", config_file.c_str());
    return false;
  }

  /**
   * @brief RViz 点击点回调
   * 
   * 当用户在 RViz 中点击点时调用
   * 用于交互式构建多边形
   */
  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Received clicked point: (%.2f, %.2f)",
                 msg->point.x, msg->point.y);

    if (manager_->getCurrentPolygonVertexCount() == 0) {
      std::string poly_name = "polygon_" + std::to_string(
          manager_->getPolygonCount() + 1);
      manager_->startCreatingPolygon(poly_name, current_drawing_type_);

      RCLCPP_INFO(this->get_logger(),
                  "Started creating polygon '%s' (type: %s)",
                  poly_name.c_str(), PolygonManager::areaTypeToString(current_drawing_type_).c_str());
    }

    size_t vertex_count = manager_->addVertexToCurrentPolygon(msg->point);

    RCLCPP_INFO(this->get_logger(),
                "Added vertex. Total: %zu (call close_polygon service to finish)",
                vertex_count);
  }

  /**
   * @brief 敌方位置回调
   */
  void onEnemyPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_enemy_pose_ = *msg;
    has_enemy_position_ = true;

    // 检测敌方状态
    std::string state = manager_->generateEnemyStateString(*msg);

    // 发布状态
    auto state_msg = std::make_unique<std_msgs::msg::String>();
    state_msg->data = state;
    enemy_state_publisher_->publish(std::move(state_msg));

    // 如果在禁区，打印警告
    if (manager_->isEnemyInForbiddenArea(*msg)) {
      RCLCPP_WARN(this->get_logger(),
                  "ALERT: Enemy in forbidden area!");
    }
  }

  /**
   * @brief 地图接收回调
   * 
   * 用于验证地图坐标系是否为 "map"
   */
  void onMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    static bool logged = false;

    if (!logged) {
      RCLCPP_INFO(this->get_logger(),
                  "Received map from frame_id: %s",
                  msg->header.frame_id.c_str());

      if (msg->header.frame_id != "map") {
        RCLCPP_WARN(this->get_logger(),
                    "WARNING: Map frame_id is '%s', expected 'map'!",
                    msg->header.frame_id.c_str());
      }

      logged = true;
    }
  }

  /**
   * @brief 区域类型选择回调
   */
  void onSetAreaType(const std_msgs::msg::String::SharedPtr msg) {
    std::string type_str = msg->data;
    AreaType new_type = PolygonManager::stringToAreaType(type_str);
    current_drawing_type_ = new_type;

    RCLCPP_INFO(this->get_logger(),
                "Area type set to: %s", type_str.c_str());
  }

  /**
   * @brief 闭合多边形服务回调
   */
  void onClosePolygon(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (manager_->getCurrentPolygonVertexCount() < 3) {
      response->success = false;
      response->message = "Need at least 3 vertices, got " +
          std::to_string(manager_->getCurrentPolygonVertexCount());
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    if (manager_->closeCurrentPolygon()) {
      response->success = true;
      response->message = "Polygon closed successfully";
      RCLCPP_INFO(this->get_logger(), "Polygon closed via service");
      saveCurrentConfig();
    } else {
      response->success = false;
      response->message = "Failed to close polygon (name conflict?)";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    }
  }

  /**
   * @brief 取消多边形服务回调
   */
  void onCancelPolygon(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (manager_->getCurrentPolygonVertexCount() == 0) {
      response->success = false;
      response->message = "No polygon being created";
      return;
    }

    manager_->cancelCurrentPolygon();
    response->success = true;
    response->message = "Polygon creation cancelled";
    RCLCPP_INFO(this->get_logger(), "Polygon cancelled via service");
  }

  /**
   * @brief 发布当前编辑状态
   */
  void publishStatus() {
    auto msg = std::make_unique<std_msgs::msg::String>();

    std::string type_str = PolygonManager::areaTypeToString(current_drawing_type_);
    size_t vertices = manager_->getCurrentPolygonVertexCount();
    size_t total = manager_->getPolygonCount();

    if (vertices > 0) {
      msg->data = "Drawing: " + type_str + " (" +
          std::to_string(vertices) + " vertices) | Total: " +
          std::to_string(total) + " polygons";
    } else {
      msg->data = "Ready (type: " + type_str + ") | Total: " +
          std::to_string(total) + " polygons";
    }

    status_publisher_->publish(std::move(msg));
  }

  /**
   * @brief 周期性发布 Marker
   */
  void publishMarkers() {
    // 发布已定义的多边形
    auto marker_array = manager_->getMarkerArray();
    marker_publisher_->publish(marker_array);

    // 如果正在创建多边形，也发布当前的顶点
    if (manager_->getCurrentPolygonVertexCount() > 0) {
      auto current_marker = manager_->getCurrentPolygonMarker();
      visualization_msgs::msg::MarkerArray current_array;
      current_array.markers.push_back(current_marker);
      marker_publisher_->publish(current_array);
    }
  }

  /**
   * @brief 从 YAML+PGM 文件加载离线地图并发布
   *
   * 参考 map_edit 的 loadAndPublishMap 实现
   * 流程：读取 YAML 获取地图参数 → 读取 PGM 像素数据 → 转换为 OccupancyGrid → 发布
   */
  void loadOfflineMap(const std::string& yaml_path) {
    try {
      RCLCPP_INFO(this->get_logger(), "Loading offline map: %s", yaml_path.c_str());

      YAML::Node config = YAML::LoadFile(yaml_path);
      std::string image_file = config["image"].as<std::string>();
      double resolution = config["resolution"].as<double>();
      auto origin = config["origin"];
      double origin_x = origin[0].as<double>();
      double origin_y = origin[1].as<double>();
      double origin_theta = origin[2].as<double>();
      int negate = config["negate"].as<int>();
      double occupied_thresh = config["occupied_thresh"].as<double>();
      double free_thresh = config["free_thresh"].as<double>();

      // 构造 PGM 完整路径
      std::filesystem::path yaml_dir = std::filesystem::path(yaml_path).parent_path();
      std::string pgm_path = std::filesystem::path(image_file).is_absolute()
          ? image_file
          : (yaml_dir / image_file).string();

      // 解析 PGM 文件
      int width = 0, height = 0;
      std::vector<uint8_t> pixels;
      if (!loadPgm(pgm_path, width, height, pixels)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PGM: %s", pgm_path.c_str());
        return;
      }

      // 构建 OccupancyGrid
      nav_msgs::msg::OccupancyGrid map_msg;
      map_msg.header.frame_id = "map";
      map_msg.header.stamp = this->now();
      map_msg.info.resolution = resolution;
      map_msg.info.width = width;
      map_msg.info.height = height;
      map_msg.info.origin.position.x = origin_x;
      map_msg.info.origin.position.y = origin_y;
      map_msg.info.origin.position.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, origin_theta);
      map_msg.info.origin.orientation.x = q.x();
      map_msg.info.origin.orientation.y = q.y();
      map_msg.info.origin.orientation.z = q.z();
      map_msg.info.origin.orientation.w = q.w();

      map_msg.data.resize(width * height);
      for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
          // PGM 行 0 是图像顶部，地图 Y 轴向上，需翻转
          int img_idx = y * width + x;
          int map_idx = (height - 1 - y) * width + x;

          double p = negate
              ? pixels[img_idx] / 255.0
              : (255 - pixels[img_idx]) / 255.0;

          if (p > occupied_thresh) {
            map_msg.data[map_idx] = 100;
          } else if (p < free_thresh) {
            map_msg.data[map_idx] = 0;
          } else {
            map_msg.data[map_idx] = -1;
          }
        }
      }

      // 缓存并发布离线地图
      cached_map_ = map_msg;
      has_offline_map_ = true;

      // 首次发布
      cached_map_.header.stamp = this->now();
      map_publisher_->publish(cached_map_);

      // 启动周期性重发（每 5 秒），确保 RViz 等后启动的节点能收到
      map_publish_timer_ = this->create_wall_timer(
          std::chrono::seconds(5),
          [this]() {
            if (has_offline_map_) {
              cached_map_.header.stamp = this->now();
              map_publisher_->publish(cached_map_);
            }
          });

      RCLCPP_INFO(this->get_logger(),
                  "Offline map published: %dx%d, resolution: %.3f",
                  width, height, resolution);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to load offline map: %s", e.what());
    }
  }

  /**
   * @brief 解析 PGM (P5 二进制) 图像文件
   */
  static bool loadPgm(const std::string& path, int& width, int& height,
                       std::vector<uint8_t>& pixels) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) return false;

    std::string magic;
    file >> magic;
    if (magic != "P5") return false;

    // 跳过注释
    char c;
    file.get(c);
    while (file.peek() == '#') {
      std::string comment;
      std::getline(file, comment);
    }

    int maxval;
    file >> width >> height >> maxval;
    file.get(c);  // 跳过 maxval 后的单字节空白

    pixels.resize(width * height);
    file.read(reinterpret_cast<char*>(pixels.data()), pixels.size());
    return file.good();
  }

  /**
   * @brief 保存当前配置
   */
  void saveCurrentConfig() {
    std::string config_file = this->get_parameter("config_file").as_string();
    if (manager_->saveToYaml(config_file)) {
      RCLCPP_INFO(this->get_logger(),
                  "Saved configuration to: %s", config_file.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to save configuration to: %s", config_file.c_str());
    }
  }
};

}  // namespace polygon_manager

/**
 * @brief 主函数
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<polygon_manager::PolygonManagerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
