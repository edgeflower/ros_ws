# sensor_scan_generation

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy%7CGalactic%7CHumble%7CIron%7CJade-brightgreen.svg)](https://docs.ros.org/en/humble/)

将地图坐标系下的配准点云数据转换为传感器坐标系点云，并生成机器人里程计和 TF 变换的 ROS2 功能包。

## 概述

本功能包专为 **PB2025 哨兵导航系统** 设计，负责在不同坐标系之间进行转换，并为机器人控制和导航生成里程计数据。

### 主要功能

- 同步滤波里程计和点云消息
- 地图坐标系到传感器坐标系的转换
- 实时速度估计（线速度和角速度）
- TF 变换广播

## 系统架构

### 坐标系关系

```
                    ┌─────────────────┐
                    │    odom/map     │  (里程计/地图系)
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │   base_link     │  (底盘)
                    │  (gimbal_yaw)   │  (机器人基座)
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │     lidar       │  (雷达)
                    └─────────────────┘
```

### 数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                              输入                                    │
├─────────────────────────────────────────────────────────────────────┤
│  lidar_odometry    - LOAM/SLAM 里程计 (odom → lidar)                │
│  registered_scan   - 地图坐标系下的配准点云                         │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    SensorScanGenerationNode                         │
├─────────────────────────────────────────────────────────────────────┤
│  1. 同步里程计和点云消息                                              │
│  2. 获取 TF 变换: lidar → base, lidar → robot_base                 │
│  3. 计算: odom → base, odom → robot_base                           │
│  4. 逆变换: map → lidar（点云转换回传感器系）                       │
│  5. 根据位姿差分估计速度                                              │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                              输出                                    │
├─────────────────────────────────────────────────────────────────────┤
│  sensor_scan       - 传感器坐标系下的点云                            │
│  odometry          - 带速度估计的机器人里程计                        │
│  tf (odom→base)    - TF 变换广播                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## 安装

### 依赖项

```
rclcpp
rclcpp_components
std_msgs
nav_msgs
sensor_msgs
pcl_ros
tf2
tf2_ros
tf2_geometry_msgs
message_filters
pcl_conversions
```

### 从源码构建

```bash
cd /path/to/ros_ws/src
git clone <repository_url>
cd ..
colcon build --packages-select sensor_scan_generation
source install/setup.bash
```

## 使用方法

### 启动参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `namespace` | string | `""` | 节点命名空间 |
| `lidar_frame` | string | `"front_mid360"` | 激光雷达坐标系 ID |
| `base_frame` | string | `"chassis"` | 底盘坐标系 ID |
| `robot_base_frame` | string | `"gimbal_yaw"` | 云台/机器人基座坐标系 ID |

### 启动节点

```bash
# 使用默认参数
ros2 launch sensor_scan_generation sensor_scan_generation.launch.py

# 自定义参数
ros2 launch sensor_scan_generation sensor_scan_generation.launch.py \
    lidar_frame:=custom_lidar \
    base_frame:=custom_base \
    robot_base_frame:=custom_gimbal
```

### 单独运行组件

```bash
ros2 run sensor_scan_generation sensor_scan_generation_node \
    --ros-args -p lidar_frame:=front_mid360 \
               -p base_frame:=chassis \
               -p robot_base_frame:=gimbal_yaw
```

## 话题列表

### 订阅话题

| 话题 | 消息类型 | QoS | 说明 |
|------|----------|-----|------|
| `lidar_odometry` | `nav_msgs/Odometry` | BEST_EFFORT | LOAM/SLAM 输出的里程计 (odom → lidar) |
| `registered_scan` | `sensor_msgs/PointCloud2` | BEST_EFFORT | 地图坐标系下的配准点云 |

### 发布话题

| 话题 | 消息类型 | QoS | 说明 |
|------|----------|-----|------|
| `sensor_scan` | `sensor_msgs/PointCloud2` | RELIABLE(2) | 传感器坐标系下的点云 |
| `odometry` | `nav_msgs/Odometry` | RELIABLE(2) | 带速度估计的机器人里程计 |

### TF 变换

| 父坐标系 | 子坐标系 | 说明 |
|----------|----------|------|
| `odom` | `base_frame` | 里程计到底盘的变换 |

## 算法详解

### 消息同步

使用 `message_filters` 的 `ApproximateTime` 策略同步里程计和点云消息，队列大小为 100。

### 坐标转换

对每一组同步消息：

```cpp
// 从 TF 树获取变换
tf_lidar_to_base = lookupTransform(base_frame, lidar_frame)
tf_lidar_to_robot_base = lookupTransform(robot_base_frame, lidar_frame)

// 从输入里程计消息获取
tf_odom_to_lidar = odometry_msg.pose.pose

// 计算组合变换
tf_odom_to_base = tf_odom_to_lidar * tf_lidar_to_base
tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base
```

### 点云转换

将地图系下的配准点云转换回传感器系：

```cpp
sensor_scan = inverse(tf_odom_to_lidar) * registered_scan
```

这对局部路径规划非常有用，因为局部规划需要传感器坐标系的数据。

### 速度估计

通过位姿的有限差分计算速度：

```cpp
dt = current_time - previous_time
linear_velocity = (current_position - previous_position) / dt
q_diff = current_orientation * previous_orientation.inverse()
angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt
```

## 应用场景

本功能包连接全局 SLAM 和局部规划：

1. **SLAM/LOAM** 产生全局地图和里程计
2. **sensor_scan_generation** 转换为传感器系点云和机器人里程计
3. **局部规划器** 使用传感器系点云进行实时避障

## 故障排查

### TF 查找失败

```
[WARN] TF lookup failed: ... Returning identity.
```

**解决方法**: 确保所有需要的 TF 变换都已发布。检查：
```bash
ros2 run tf2_tools view_frames
```

### 无输出消息

**检查**: 输入话题是否正在发布
```bash
ros2 topic hz /lidar_odometry
ros2 topic hz /registered_scan
```

**检查**: 坐标系 ID 是否与 TF 树匹配
```bash
ros2 run tf2_ros tf2_echo odom front_mid360
```

## 许可证

Apache License 2.0 - 详见 [LICENSE](LICENSE)

## 作者

Lihan Chen (lihanchen2004@163.com)

## 相关功能包

- `loam_interface` - LOAM SLAM 集成
- `fast_planner` - 局部轨迹规划
- `pb2025_nav_bringup` - 导航栈配置
