# icp_registration

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-brightgreen.svg)](https://docs.ros.org/en/humble/)

基于 ICP (Iterative Closest Point) 算法的激光雷达定位与重定位功能包，专为 PB2025 哨兵导航系统设计。

## 概述

本功能包实现了**多候选位姿 + 两阶段法向量增强 ICP** 的定位算法，能够将实时激光扫描数据与预先生成的全局地图进行匹配，实现机器人在已知地图中的精确定位和重定位。

### 核心特性

- **多候选位姿生成** - XY 平面网格搜索 + Yaw 多角度采样
- **两阶段 ICP 匹配** - 粗匹配快速筛选 + 精匹配精确优化
- **法向量增强** - 使用点云法向量提高匹配鲁棒性和精度
- **高频 TF 发布** - 独立线程 50Hz 发布 map→odom 变换
- **自动初始化** - 首帧点云自动触发定位
- **RViz 集成** - 支持 `/initialpose` 话题手动重定位

## 系统架构

### 坐标系关系

```
                    ┌─────────────────┐
                    │      map        │  (全局地图系)
                    └────────┬────────┘
                             │  map→odom (本节点发布)
                             │
                    ┌────────▼────────┐
                    │      odom       │  (里程计系)
                    └────────┬────────┘
                             │  odom→base (里程计发布)
                             │
                    ┌────────▼────────┐
                    │     base        │  (机器人基座)
                    └────────┬────────┘
                             │  base→lidar (静态TF)
                             │
                    ┌────────▼────────┐
                    │     lidar       │  (激光雷达)
                    └─────────────────┘
```

### 算法流程

```
┌─────────────────────────────────────────────────────────────────────┐
│                              输入                                    │
├─────────────────────────────────────────────────────────────────────┤
│  registered_scan    - 实时激光点云 (odom 系或 map 系)                │
│  /initialpose       - 初始位姿估计 (RViz 2D Pose Estimate)           │
│  prior_pcd_file     - 预加载的地图点云文件 (.pcd)                    │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   候选位姿生成 (Multi-Align)                         │
├─────────────────────────────────────────────────────────────────────┤
│  在初始位姿周围生成候选位姿:                                          │
│    - XY 平面: ±N × xy_offset, N = xy_search_steps                  │
│    - Yaw 角度: ±K × yaw_resolution, K = yaw_offset / resolution    │
│  例如: N=3, offset=0.5m → 7×7=49 个 XY 候选点                      │
│       K=3, resolution=10° → 7 个 Yaw 候选角                         │
│       总候选数 ≈ 49 × 7 = 343                                        │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      粗匹配 (Rough Match)                            │
├─────────────────────────────────────────────────────────────────────┤
│  • 降采样: rough_leaf_size (默认 0.4m)                              │
│  • 目标地图: 粗降采样地图                                            │
│  • 特征: XYZI + 法向量 (PointNormal)                                │
│  • 迭代次数: 10 次                                                  │
│  • 对所有候选位姿执行 ICP，选择 fitness score 最小的                 │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      精匹配 (Refine Match)                           │
├─────────────────────────────────────────────────────────────────────┤
│  • 降采样: refine_leaf_size (默认 0.1m)                             │
│  • 目标地图: 精降采样地图                                            │
│  • 特征: XYZI + 法向量 (PointNormal)                                │
│  • 迭代次数: 5 次                                                   │
│  • 初始位姿: 粗匹配的最佳结果                                        │
│  • 收敛判定: fitness score < thresh (默认 0.1~0.15)                 │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        TF 变换计算                                   │
├─────────────────────────────────────────────────────────────────────┤
│  T_map_odom = T_map_lidar_refined                                    │
│                                                                        │
│  以 50Hz 高频发布，时间戳外推 +0.1s 防止下游节点超时                  │
└─────────────────────────────────────────────────────────────────────┘
```

## 安装

### 依赖项

```
rclcpp
rclcpp_components
sensor_msgs
pcl_conversions
tf2
tf2_ros
tf2_geometry_msgs
tf2_eigen
geometry_msgs
livox_ros_driver2
eigen
```

### 从源码构建

```bash
cd ~/ros_ws/src
git clone <repository_url>
cd ~/ros_ws
colcon build --packages-select icp_registration
source install/setup.bash
```

## 使用方法

### 配置文件

编辑 `config/icp.yaml`:

```yaml
icp_registration:
  ros__parameters:
    # 点云降采样参数
    rough_leaf_size: 0.4      # 粗匹配降采样尺寸 (米)
    refine_leaf_size: 0.1     # 精匹配降采样尺寸 (米)

    # 地图文件路径
    prior_pcd_file: "/path/to/your/map.pcd"

    # 坐标系配置
    map_frame_id: "map"
    odom_frame_id: "odom"
    robot_base_frame: "base_link"
    laser_frame_id: "front_mid360"

    # 输入话题
    pointcloud_topic: "/livox/lidar/pointcloud"  # 或 "registered_scan"

    # ICP 搜索参数
    xy_search_steps: 3       # XY 搜索范围 (±N)
    xy_offset: 0.5           # XY 搜索步长 (米)
    yaw_offset: 30.0         # Yaw 搜索范围 (度)
    yaw_resolution: 10.0     # Yaw 搜索步长 (度)

    # 收敛阈值
    thresh: 0.15             # ICP fitness score 阈值

    # 初始位姿 [x, y, z, roll, pitch, yaw]
    initial_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 启动节点

```bash
# 使用默认配置
ros2 launch icp_registration icp.launch.py

# 或直接运行节点
ros2 run icp_registration icp_registration_node --ros-args --params-file config/icp.yaml
```

### 手动重定位

使用 RViz 设置初始位姿:

1. 启动 RViz: `rviz2`
2. 添加 **2D Pose Estimate** 工具
3. 在地图上点击并拖动设置位置和朝向
4. 或发布话题:

```bash
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
  "{
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: 1.0, y: 2.0, z: 0.0},
        orientation: {w: 1.0}
      }
    }
  }"
```

## 话题列表

### 订阅话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `pointcloud_topic` | `sensor_msgs/PointCloud2` | 实时激光点云数据 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初始位姿估计 |

### TF 变换

| 变换 | 发布频率 | 说明 |
|------|----------|------|
| `map → odom` | 50 Hz | 全局定位结果 |

## 参数详解

### 点云处理参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `rough_leaf_size` | double | 0.4 | 粗匹配降采样尺寸，值越大匹配越快但精度越低 |
| `refine_leaf_size` | double | 0.1 | 精匹配降采样尺寸，影响最终定位精度 |
| `prior_pcd_file` | string | - | 预加载地图的 PCD 文件路径 |

### 坐标系参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `map_frame_id` | string | "map" | 全局地图坐标系 |
| `odom_frame_id` | string | "odom" | 里程计坐标系 |
| `robot_base_frame` | string | "base_link" | 机器人基座坐标系 |
| `laser_frame_id` | string | "front_mid360" | 激光雷达坐标系 |

### ICP 搜索参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `xy_search_steps` | int | 3 | XY 搜索范围，候选数 = (2N+1)² |
| `xy_offset` | double | 0.2~1.0 | XY 搜索步长（米） |
| `yaw_offset` | double | 30.0 | Yaw 搜索范围（度） |
| `yaw_resolution` | double | 10.0 | Yaw 搜索步长（度） |

**候选数计算示例**:
- `xy_search_steps=3, xy_offset=0.5` → 7×7 = 49 个 XY 候选
- `yaw_offset=30, yaw_resolution=10` → 7 个 Yaw 候选
- 总候选数 ≈ 343

### 收敛判定参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `thresh` | double | 0.15 | ICP fitness score 收敛阈值，越小越严格 |
| `initial_pose` | double[6] | [0,0,0,0,0,0] | 初始位姿 [x,y,z,roll,pitch,yaw] |

## 算法详解

### 法向量计算

使用 PCL 的 `NormalEstimation` 计算点云法向量:

```cpp
pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
ne.setKSearch(15);  // 使用 15 个近邻点
ne.compute(*normals);
```

法向量增强的 ICP 对平面结构（如墙壁、地面）更鲁棒。

### 时间戳外推

为防止下游节点 TF 查找超时，发布时间戳外推 0.1 秒:

```cpp
map_to_odom_.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
```

### 自动初始化

首帧点云到达时，自动使用参数中的 `initial_pose` 触发定位:

```cpp
if (first_scan_) {
    initialPoseCallback(default_pose_msg);
    first_scan_ = false;
}
```

## 性能调优

### 匹配速度 vs 精度

| 场景 | rough_leaf_size | refine_leaf_size | xy_search_steps |
|------|-----------------|------------------|-----------------|
| 快速重定位 | 0.6 | 0.2 | 2 |
| 标准配置 | 0.4 | 0.1 | 3 |
| 高精度定位 | 0.3 | 0.05 | 4 |

### 搜索范围调整

- **已知大致位置**: 减小 `xy_search_steps` 和 `yaw_offset`
- **完全丢失**: 增大搜索范围，但计算时间显著增加

## 故障排查

### 地图加载失败

```
[ERROR] Invalid PCD path: /path/to/map.pcd
```

**解决**: 检查文件路径和权限

### ICP 匹配失败

```
[ERROR] ICP Refinement failed.
```

**可能原因**:
1. 初始位姿偏差过大，超出搜索范围
2. 环境变化与地图不匹配
3. 点云质量不佳

**解决方法**:
- 增大 `xy_search_steps` 和 `yaw_offset`
- 在 RViz 中手动设置更准确的初始位姿

### TF 超时

```
[WARN] TF lookup failed: ...
```

**解决**: 确保 odom→base 的 TF 正常发布

## 应用场景

- **比赛启动定位** - 机器人在已知地图中确定初始位置
- ** kidnapped 恢复** - 机器人被移动后重新定位
- **全局漂移校正** - 弥补 LOAM/里程计的累积漂移
- **多机器人定位** - 各机器人在同一地图中独立定位

## 许可证

TODO: License declaration

## 作者

zcf (baiyeweiguang@163.com)

## 相关功能包

- `loam_interface` - LOAM SLAM，提供实时里程计
- `sensor_scan_generation` - 坐标系转换和点云生成
- `fast_planner` - 局部路径规划
