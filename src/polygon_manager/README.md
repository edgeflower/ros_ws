# RoboMaster 哨兵禁区管理系统

## 项目概述

这是一个完整的 ROS2 禁区管理系统，为 RoboMaster 哨兵提供：

- **实时地图支持**：支持 SLAM 在线建图和离线地图
- **交互式区域编辑**：通过 RViz 点击创建多边形区域
- **敌方检测**：实时检测敌方是否进入禁区
- **可视化显示**：在 RViz 中实时显示所有定义的区域
- **配置管理**：YAML 格式保存和加载配置
- **BehaviorTree 集成**：支持在行为树中使用区域检测条件

## 技术栈

- **操作系统**：Ubuntu 22.04 LTS
- **中间件**：ROS2 Humble
- **编程语言**：C++ 17
- **导航**：nav2，slam_toolbox
- **可视化**：RViz2
- **行为树**：BehaviorTree.CPP v4
- **配置格式**：YAML

## 系统架构

### 组件说明

```
┌─────────────────────────────────────────────────────────┐
│         Polygon Manager Node (ROS2)                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │  PolygonManager (Core Logic)                     │  │
│  │  ├─ Polygon Management (CRUD)                    │  │
│  │  ├─ Enemy Detection (Point-in-Polygon)           │  │
│  │  ├─ Configuration I/O (YAML)                     │  │
│  │  └─ Marker Generation                            │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
        │         │           │           │
        ├─────────┴─┬─────────┴─┬─────────┤
        │           │           │         │
    ┌────────┐ ┌─────────┐ ┌────────┐ ┌──────────┐
    │RViz    │ │slam/map │ │Enemy   │ │RViz      │
    │Points  │ │Topic    │ │Position│ │Markers   │
    │Publish │ │/map     │ │Topic   │ │Display   │
    └────────┘ └─────────┘ └────────┘ └──────────┘
```

### 关键类

1. **PolygonManager** (polygon_manager.hpp/cpp)
   - 核心逻辑类，不依赖 ROS
   - 管理多边形、敌方检测、配置 I/O

2. **GeometryUtils** (geometry_utils.hpp/cpp)
   - 几何算法实现
   - 射线法点在多边形内判定
   - O(n) 时间复杂度

3. **MarkerUtils** (marker_utils.hpp/cpp)
   - RViz 可视化工具
   - 生成 Marker 消息

4. **PolygonManagerNode** (polygon_node.cpp)
   - ROS2 节点
   - 订阅/发布消息
   - 交互式 polygon 创建

5. **EnemyInAreaCondition** (bt_enemy_in_area.cpp)
   - BehaviorTree.CPP v4 节点
   - 在行为树中使用区域检测

## 编译安装

### 前置要求

```bash
# Ubuntu 22.04 上安装依赖
sudo apt-get update
sudo apt-get install ros-humble-slam-toolbox
sudo apt-get install ros-humble-nav2
sudo apt-get install ros-humble-rviz2
sudo apt-get install libyaml-cpp-dev
sudo apt-get install ros-humble-behaviortree-cpp-v4
```

### 编译

```bash
# 创建工作空间（如果还没有）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 复制项目
cp -r polygon_manager src/

# 编译
colcon build --packages-select polygon_manager

# 加载环境
source install/setup.bash
```

## 使用方法

### 1. 启动节点

```bash
ros2 launch polygon_manager polygon_manager.launch.py
```

#### 启动参数

```bash
# 指定配置文件
ros2 launch polygon_manager polygon_manager.launch.py \
  config_file:=/path/to/areas.yaml

# 设置更新频率（Hz）
ros2 launch polygon_manager polygon_manager.launch.py \
  update_frequency:=20.0

# 禁用 RViz（如果只需要后台处理）
ros2 launch polygon_manager polygon_manager.launch.py \
  use_rviz:=false
```

### 2. 交互式创建多边形

启动后在 RViz 中：

1. 选择工具栏中的 "Publish Point"
2. 在地图上依次点击多边形的各个顶点
3. 当点击 4 个顶点后，系统会自动闭合多边形
4. 配置自动保存到 YAML 文件

**关键 RViz 工具**：
- "Publish Point"：发送点击的坐标
- 确保将消息发布到 `/clicked_point` topic

### 3. 加载预定义配置

编辑 `config/areas.yaml`：

```yaml
areas:
  - name: forbidden_zone_1
    type: forbidden
    points:
      - [1.0, 1.0]
      - [5.0, 1.0]
      - [5.0, 4.0]
      - [1.0, 4.0]
```

类型选项：
- `forbidden`：禁区（红色）
- `protect`：防守区（绿色）
- `patrol`：巡逻区（蓝色）
- `attack`：攻击区（黄色）

### 4. 监听敌方状态

```bash
# 在另一个终端查看敌方状态
ros2 topic echo /enemy_area_state
```

## ROS Topic 说明

### 订阅 Topics

| Topic | 类型 | 来源 | 说明 |
|-------|------|------|------|
| `/clicked_point` | geometry_msgs/PointStamped | RViz | RViz 中点击的坐标 |
| `/enemy_position` | geometry_msgs/PoseStamped | 敌方感知模块 | 敌方实时位置 |
| `/map` | nav_msgs/OccupancyGrid | slam/nav2 | 地图数据（用于坐标系验证） |

### 发布 Topics

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/polygon_areas/visualization_marker_array` | visualization_msgs/MarkerArray | 10Hz | 所有多边形的可视化 |
| `/enemy_area_state` | std_msgs/String | 同敌方位置 | 敌方所在区域状态 |

## 坐标系说明

### 为什么使用 map 而不是 odom？

**odom 坐标系**：
- 以机器人初始位置为原点
- 随机器人移动而相对变化
- 短期定位，局部坐标系
- **问题**：禁区是地图绝对约束，不应随机器人移动

**map 坐标系**：
- 以地图固定位置为原点
- 始终保持全局一致
- 长期定位，全局坐标系
- **适用**：地图约束、全局导航、绝对位置定义

### SLAM vs 离线地图

两者都使用 map 坐标系：

```
slam_toolbox:                nav2_map_server:
  /map (OccupancyGrid)         /map (OccupancyGrid)
  frame_id: "map"              frame_id: "map"
```

**统一设计**：禁区定义在 map 坐标系下，无论哪种地图来源都能使用。

## 几何算法详解

### 射线法（Ray Casting Algorithm）

#### 原理

从目标点发射一条射线（向右），统计与多边形边界的交点数：

- **交点数为奇数** → 点在多边形内
- **交点数为偶数** → 点在多边形外

#### 数学推导

```
设多边形 P = {v0, v1, ..., vn}
点 Q = (x, y)

从 Q 向右发射射线：{(x+t, y) | t > 0}

对每条边 (vi, vi+1)：
  1. 检查边的 y 范围是否包含 Q.y
  2. 计算交点 x 坐标
  3. 如果交点在 Q.x 右侧，计数 +1

结果：count % 2 == 1 → 内部，否则外部
```

#### 边界处理

1. **浮点精度**：使用 EPSILON = 1e-10
2. **顶点处理**：只计算一端的交点，避免重复
3. **边界上的点**：检查到边的距离 < EPSILON

#### 时间复杂度

- **O(n)**：需要遍历所有 n 条边
- **支持凸和凹多边形**
- **无需预处理**

### 实现示例

```cpp
// 检查点 (2.5, 2.5) 是否在矩形 [(1,1), (5,1), (5,4), (1,4)] 内
std::vector<Point> rect = {{1,1}, {5,1}, {5,4}, {1,4}};
bool inside = GeometryUtils::isPointInPolygon(2.5, 2.5, rect);
// 结果：true
```

## BehaviorTree 集成

### 使用方式

#### 1. 在 C++ 中注册节点

```cpp
#include "polygon_manager/polygon_manager.hpp"
#include "polygon_manager/bt_enemy_in_area.cpp"

int main() {
    auto manager = std::make_shared<PolygonManager>();
    manager->loadFromYaml("config/areas.yaml");
    
    // 设置全局管理器实例
    polygon_manager::EnemyInAreaCondition::setManager(manager.get());
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<polygon_manager::EnemyInAreaCondition>(
        "EnemyInArea");
    
    // 加载和执行树
    auto tree = factory.createTreeFromFile("my_tree.xml");
    tree.tickOnce();
}
```

#### 2. XML 定义行为树

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <!-- 检查敌方是否在禁区 -->
      <EnemyInArea enemy_pose="${enemy_pose}"
                   area_name="forbidden_zone_1"/>
      
      <!-- 如果在禁区，执行警告 -->
      <AlertAction/>
    </Sequence>
  </BehaviorTree>
</root>
```

#### 3. 节点返回值

- `SUCCESS`：敌方在指定区域内
- `FAILURE`：敌方不在指定区域内
- `RUNNING`：不使用（Condition Node）

### 参数说明

| 参数 | 类型 | 说明 | 必需 |
|------|------|------|------|
| `enemy_pose` | geometry_msgs/PoseStamped | 敌方位置 | ✓ |
| `area_name` | string | 区域名称（如 "forbidden_zone_1"） | ✗ |
| `area_type` | string | 区域类型（forbidden/protect/patrol/attack） | ✗ |

注：`area_name` 和 `area_type` 二者择一

## 配置文件格式

### areas.yaml 示例

```yaml
areas:
  # 禁区（矩形）
  - name: forbidden_zone_1
    type: forbidden
    points:
      - [1.0, 1.0]
      - [5.0, 1.0]
      - [5.0, 4.0]
      - [1.0, 4.0]

  # 防守区（矩形）
  - name: protect_zone_1
    type: protect
    points:
      - [7.0, 2.0]
      - [9.0, 2.0]
      - [9.0, 5.0]
      - [7.0, 5.0]

  # 复杂形状（凹多边形）
  - name: complex_zone
    type: patrol
    points:
      - [0.0, 0.0]
      - [3.0, 0.0]
      - [3.0, 1.0]
      - [2.0, 1.0]
      - [2.0, 2.0]
      - [0.0, 2.0]
```

## 常见问题

### Q1: 如何验证地图坐标系？

```bash
# 查看 /map topic 的 frame_id
ros2 topic info /map

# 应该显示：
# frame_id: "map"
```

### Q2: 如何在不同的地图间切换？

1. 停止当前节点
2. 编辑 `areas.yaml` 中的坐标（根据新地图调整）
3. 重新启动节点

系统会自动加载新配置。

### Q3: 点在多边形边界上时的判定？

算法将距离 < EPSILON 的点视为"在边上"，返回 true（在多边形内）。

### Q4: 支持的最大多边形顶点数？

理论上无限制，但过多顶点会增加计算时间（O(n)）。
建议 < 100 个顶点。

### Q5: 如何调试几何算法？

```cpp
// 启用 ROS 调试日志
export ROS_LOG_LEVEL=polygon_manager:=DEBUG
ros2 run polygon_manager polygon_manager_node
```

## 开发指南

### 添加新的区域类型

1. 在 `polygon_area.hpp` 中的 `AreaType` enum 添加新类型
2. 在 `marker_utils.cpp` 的 `getColorByType()` 中添加颜色映射
3. 在 `polygon_manager.cpp` 中更新字符串转换函数

### 自定义 Marker 样式

编辑 `marker_utils.cpp` 中的常数：
- `LINE_WIDTH`：改变线条宽度
- `getColorByType()`：改变颜色方案

### 扩展检测功能

在 `PolygonManager` 中添加新方法：

```cpp
// 检查多边形是否与圆形相交
bool doesPolygonIntersectCircle(
    uint32_t polygon_id,
    double center_x, double center_y, double radius);
```

## 性能指标

- **点在多边形检测**：O(n)，其中 n = 顶点数
- **发布频率**：可配置，默认 10 Hz
- **实时性**：< 10 ms 处理延迟（4个顶点多边形）
- **内存**：< 1 MB（100 个多边形）

## 故障排除

### 问题：编译失败

```bash
# 确保安装了所有依赖
rosdep install --from-paths src --ignore-src -y

# 清除构建缓存
rm -rf build install

# 重新编译
colcon build --packages-select polygon_manager
```

### 问题：RViz 中不显示 Marker

1. 检查 `/polygon_areas/visualization_marker_array` topic 是否有数据
   ```bash
   ros2 topic echo /polygon_areas/visualization_marker_array
   ```

2. 在 RViz 中添加 MarkerArray 显示：
   - Add → By topic → polygon_areas/visualization_marker_array

3. 确认 `frame_id` 设置为 "map"

### 问题：敌方位置检测不工作

1. 检查 `/enemy_position` topic 是否发布：
   ```bash
   ros2 topic echo /enemy_position
   ```

2. 验证坐标系：敌方位置的 `frame_id` 应该是 "map"

3. 检查多边形是否有效：
   ```bash
   ros2 service call /polygon_manager_debug get_polygons
   ```

## 许可证

Apache License 2.0

## 技术支持

如有问题，请查看：
- 源代码注释（详细的实现说明）
- 本文档各部分
- ROS2 官方文档：https://docs.ros.org/

## 更新日志

### v1.0.0 (2025-05)
- 初始版本
- 完整的多边形管理系统
- SLAM 和离线地图支持
- BehaviorTree 集成
- YAML 配置持久化
