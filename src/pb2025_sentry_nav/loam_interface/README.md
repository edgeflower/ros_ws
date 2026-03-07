# LOAM Interface

一个 ROS2 坐标系转换适配器，用于将 LOAM (Lidar Odometry and Mapping) 的输出从 `lidar_odom` 坐标系转换到全局 `odom` 坐标系。

## 功能概述

LOAM 算法通常以激光雷达自身为中心建立一个 `lidar_odom` 坐标系进行里程计估计和点云配准。然而，在实际机器人系统中，我们需要将激光雷达的数据统一到全局坐标系（如 `odom`）中，以便与导航栈、状态估计器等其他模块协同工作。

`loam_interface` 包充当了这一桥梁角色：

- **订阅** LOAM 输出的里程计和配准后的点云（基于 `lidar_odom` 坐标系）
- **转换** 到全局 `odom` 坐标系
- **发布** 供下游模块使用

## 坐标变换关系

```sh
                    tf_odom_to_lidar_odom_ (初始化时从 TF 获取)
    odom ───────────────────────────────────────────> lidar_odom
                                                    |
                                                    | msg->pose.pose
                                                    v
                                                 lidar_frame
```

变换公式：`tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar`

## ROS 话题接口

### 订阅话题 (Subscribed Topics)

| 话题名称 | 消息类型 | 坐标系 | 说明 |
|---------|---------|--------|------|
| `aft_mapped_to_init` | `nav_msgs/Odometry` | `lidar_odom` | LOAM 输出的里程计估计 |
| `cloud_registered` | `sensor_msgs/PointCloud2` | `lidar_odom` | LOAM 配准后的点云 |

### 发布话题 (Published Topics)

| 话题名称 | 消息类型 | 坐标系 | 说明 |
|---------|---------|--------|------|
| `lidar_odometry` | `nav_msgs/Odometry` | `odom` | 转换到全局坐标系的里程计 |
| `registered_scan` | `sensor_msgs/PointCloud2` | `odom` | 转换到全局坐标系的点云 |

## 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|-------|------|--------|------|
| `state_estimation_topic` | string | *必填* | LOAM 里程计话题名称 |
| `registered_scan_topic` | string | *必填* | LOAM 配准点云话题名称 |
| `odom_frame` | string | `"odom"` | 全局里程计坐标系 |
| `base_frame` | string | `"base_footprint"` | 机器人基准坐标系（如云台坐标系） |
| `lidar_frame` | string | `"front_mid360"` | 激光雷达坐标系 |

## TF 坐标系要求

节点运行时需要 TF 树中存在以下变换：

- `base_frame` → `lidar_frame` 的静态变换

该变换在首次收到里程计消息时被查询并缓存。

## 使用方法

### 通过 Launch 文件启动

```bash
ros2 launch loam_interface loam_interface_launch.py
```

### 自定义命名空间

```bash
ros2 launch loam_interface loam_interface_launch.py namespace:=my_robot
```

### 在代码中使用

```python
from launch_ros.actions import Node

loam_interface = Node(
    package="loam_interface",
    executable="loam_interface_node",
    name="loam_interface",
    parameters=[{
        "state_estimation_topic": "aft_mapped_to_init",
        "registered_scan_topic": "cloud_registered",
        "odom_frame": "odom",
        "base_frame": "base_footprint",
        "lidar_frame": "front_mid360",
    }]
)
```

## 工作流程

1. **初始化阶段**
   - 声明并读取所有配置参数
   - 创建 TF Buffer 和 Listener
   - 创建发布者和订阅者

2. **首次里程计回调**
   - 通过 TF 查询 `base_frame` 到 `lidar_frame` 的变换
   - 将该变换保存为 `tf_odom_to_lidar_odom_`
   - 标记初始化完成

3. **里程计转换**
   - 提取输入里程计的位姿（`lidar_odom` → `lidar`）
   - 组合变换：`odom → lidar = odom → lidar_odom × lidar_odom → lidar`
   - 发布转换后的里程计消息

4. **点云转换**
   - 使用 `pcl_ros::transformPointCloud` 将点云转换到 `odom` 坐标系
   - 发布转换后的点云

## 依赖

- `rclcpp` - ROS2 C++ 客户端库
- `rclcpp_components` - ROS2 组件支持
- `tf2`, `tf2_ros`, `tf2_geometry_msgs` - 坐标变换
- `nav_msgs` - 里程计消息类型
- `sensor_msgs` - 传感器消息类型
- `pcl_ros` - 点云库 ROS 封装

## 构建方法

```bash
cd /path/to/ros_ws
colcon build --packages-select loam_interface
source install/setup.bash
```

## 典型应用场景

```
┌─────────────┐     aft_mapped_to_init     ┌──────────────┐
│             │ ────────────────────────> │              │
│    LOAM     │                            │ LOAM         │ lidar_odometry
│  (前端算法)  │     cloud_registered       │ Interface    │ ──────────────> ┐
│             │ ────────────────────────> │              │                  │
└─────────────┘                            └──────────────┘                  │
       │                                                                          │
       │ lidar_odom                                                               │
       ▼                                                                          ▼
┌──────────────────────────────────────────────────────────────────────────────────┐
│                              全局 odom 坐标系                                    │
│                                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │
│  │   Navigation │  │  State Est.  │  │   Mapping    │  │  Controller  │        │
│  │     Stack    │  │   (EKF/UKF)  │  │   (SLAM)     │  │              │        │
│  └──────────────┘  └──────────────┘  └──────────────┘  └──────────────┘        │
└──────────────────────────────────────────────────────────────────────────────────┘
```

## 许可证

Apache-2.0

## 作者

Lihan Chen (lihanchen2004@163.com)
