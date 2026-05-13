# 项目结构说明

## 完整的目录树

```
polygon_manager/
├── CMakeLists.txt                          # Cmake 构建配置
├── package.xml                             # ROS2 包清单
├── README.md                               # 完整文档（详细）
├── QUICK_START.md                          # 快速开始指南
├── PROJECT_STRUCTURE.md                    # 本文件
├── test_demo.py                            # Python 测试脚本
│
├── include/polygon_manager/                # 头文件目录
│   ├── polygon_area.hpp                    # 区域数据结构和枚举
│   ├── geometry_utils.hpp                  # 几何算法工具类
│   ├── marker_utils.hpp                    # RViz Marker 工具类
│   └── polygon_manager.hpp                 # 核心管理类
│
├── src/                                    # 源文件目录
│   ├── polygon_manager.cpp                 # 核心管理类实现
│   ├── geometry_utils.cpp                  # 几何算法实现
│   ├── marker_utils.cpp                    # RViz Marker 实现
│   ├── polygon_node.cpp                    # ROS2 节点实现
│   └── bt_enemy_in_area.cpp                # BehaviorTree 节点
│
├── config/                                 # 配置文件目录
│   ├── areas.yaml                          # 多边形区域配置（示例）
│   └── polygon_manager.rviz                # RViz 工作区配置
│
└── launch/                                 # 启动文件目录
    └── polygon_manager.launch.py           # ROS2 Launch 文件
```

## 文件详细说明

### 构建和包管理

#### CMakeLists.txt
- **作用**：指定如何编译项目
- **包含**：
  - 依赖库查找（rclcpp, geometry_msgs, etc.）
  - 库编译（polygon_manager_lib）
  - 节点编译（polygon_manager_node）
  - BehaviorTree 插件编译（bt_enemy_in_area）
  - 文件安装规则
- **编辑时机**：添加新库或新可执行文件时

#### package.xml
- **作用**：ROS2 包元数据
- **包含**：
  - 包名、版本、描述
  - 维护者和许可证信息
  - 编译和运行时依赖
  - 构建系统类型（ament_cmake）
- **编辑时机**：添加新的依赖库时

### 文档

#### README.md
- **详细程度**：完整的技术文档
- **包含内容**：
  - 项目概述和架构
  - 编译安装说明
  - 使用教程
  - API 文档
  - 几何算法详解
  - BehaviorTree 集成指南
  - 常见问题
  - 故障排除
  - 性能指标
- **目标读者**：希望深入了解系统的开发者

#### QUICK_START.md
- **详细程度**：快速入门
- **包含内容**：
  - 5 分钟快速编译运行
  - 常用命令
  - 简单的故障排除
- **目标读者**：想快速上手的新用户

#### PROJECT_STRUCTURE.md
- **本文件**
- **说明**：项目目录结构和各文件作用

### 头文件（include/polygon_manager/）

这些文件定义了系统的接口和数据结构，**不包含实现**。

#### polygon_area.hpp
```cpp
enum class AreaType {
  FORBIDDEN_AREA,  // 禁区
  PROTECT_AREA,    // 防守区
  PATROL_AREA,     // 巡逻区
  ATTACK_AREA      // 攻击区
};

struct PolygonArea {
  uint32_t id;
  std::string name;
  AreaType type;
  std::vector<geometry_msgs::msg::Point> vertices;
};
```
- **作用**：定义多边形区域的数据结构
- **编辑**：添加新区域类型时修改 AreaType enum

#### geometry_utils.hpp
```cpp
class GeometryUtils {
  static bool isPointInPolygon(
      double x, double y,
      const std::vector<geometry_msgs::msg::Point>& polygon);
  // ... 其他几何函数
};
```
- **作用**：几何算法工具类
- **核心算法**：射线法点在多边形内判定（O(n)）
- **包含详细注释**：算法原理、边界处理、浮点精度

#### marker_utils.hpp
```cpp
class MarkerUtils {
  static visualization_msgs::msg::Marker 
    createMarkerFromPolygon(const PolygonArea& area);
  // ... 其他可视化函数
};
```
- **作用**：RViz Marker 生成工具
- **功能**：多边形转换为 Marker、颜色管理、ID 管理

#### polygon_manager.hpp
```cpp
class PolygonManager {
  // 多边形管理
  bool addPolygon(...);
  bool deletePolygon(...);
  
  // 敌方检测
  bool isEnemyInForbiddenArea(...);
  std::vector<AreaType> getEnemyAreas(...);
  
  // 配置管理
  bool loadFromYaml(...);
  bool saveToYaml(...);
};
```
- **作用**：核心逻辑管理类
- **特点**：纯 C++ 实现，不依赖 ROS（易于测试和复用）
- **包含详细注释**：每个函数的功能、参数、返回值

### 源文件（src/）

这些文件包含了上述头文件中声明的实现。

#### geometry_utils.cpp
- **行数**：~350 行
- **内容**：
  - 射线法实现
  - 边界情况处理
  - 浮点精度处理
  - 详细的算法注释和原理解释

#### marker_utils.cpp
- **行数**：~200 行
- **内容**：
  - Marker 生成逻辑
  - 颜色映射表
  - 多边形闭合实现

#### polygon_manager.cpp
- **行数**：~450 行
- **内容**：
  - 多边形 CRUD 操作
  - 敌方检测逻辑
  - YAML 配置加载/保存
  - 索引管理

#### polygon_node.cpp
- **行数**：~250 行
- **内容**：
  - ROS2 节点类 (PolygonManagerNode)
  - Topic 订阅/发布
  - 消息回调处理
  - 配置文件加载
  - 交互式 polygon 创建逻辑

#### bt_enemy_in_area.cpp
- **行数**：~300 行
- **内容**：
  - BehaviorTree 条件节点
  - 与 BehaviorTree.CPP v4 框架集成
  - 节点状态管理
  - 输入端口定义

### 配置文件（config/）

#### areas.yaml
- **格式**：YAML
- **示例**：
  ```yaml
  areas:
    - name: forbidden_zone_1
      type: forbidden
      points: [[1, 1], [5, 1], [5, 4], [1, 4]]
  ```
- **编辑**：
  - 手动编辑来定义区域
  - 系统运行时自动保存新建的区域
  - 启动时自动加载

#### polygon_manager.rviz
- **格式**：RViz 配置
- **包含**：
  - Display 设置（Map, MarkerArray, etc.）
  - Tool 设置（PublishPoint）
  - 视图参数
  - 面板布局
- **编辑**：
  - 在 RViz GUI 中调整后自动保存
  - 或手动修改此文件

### 启动文件（launch/）

#### polygon_manager.launch.py
- **格式**：Python
- **作用**：ROS2 Launch 文件
- **功能**：
  - 参数声明
  - 节点配置
  - RViz 启动
  - Topic remapping
- **使用**：
  ```bash
  ros2 launch polygon_manager polygon_manager.launch.py \
    config_file:=/custom/path/areas.yaml \
    update_frequency:=20.0
  ```

### 测试脚本

#### test_demo.py
- **格式**：Python 3
- **作用**：系统测试和演示
- **测试项**：
  1. 点在多边形内判定
  2. 敌方检测流程
  3. Marker 可视化
  4. 性能基准测试
- **运行**：
  ```bash
  python3 test_demo.py all
  python3 test_demo.py point_in_polygon
  ```

## 代码组织原则

### 分层设计

```
应用层
  └─ PolygonManagerNode (ROS2 节点)
       ↓ 依赖
逻辑层
  └─ PolygonManager (核心管理)
       ↓ 使用
工具层
  ├─ GeometryUtils (几何算法)
  └─ MarkerUtils (可视化)
       ↓ 使用
数据层
  └─ PolygonArea (数据结构)
```

**优点**：
- 关注点分离
- 易于测试（纯 C++ 逻辑可独立测试）
- 易于复用（可在非 ROS 项目中使用核心逻辑）

### 头源分离

- **头文件**：接口定义、常量、内联函数
- **源文件**：实现细节
- **优点**：
  - 编译时间更快（只需重编改动的源文件）
  - 代码清晰（接口和实现分开）
  - 易于维护

### 注释规范

- **类级注释**：说明作用和使用方式
- **函数注释**：
  - 目的
  - 参数说明
  - 返回值
  - 注意事项
- **算法注释**：
  - 原理解释
  - 时间复杂度
  - 边界情况处理
- **工业级代码**：详细注释便于初学者理解

## 编译过程

```
CMakeLists.txt
      ↓
找到依赖库 (find_package)
      ↓
编译库
  ├─ polygon_manager_lib (静态库)
  └─ src/*.cpp
      ↓
编译可执行文件
  ├─ polygon_manager_node (链接 polygon_manager_lib)
  └─ bt_enemy_in_area (动态库)
      ↓
安装目标
  ├─ 可执行文件 → lib/polygon_manager/
  ├─ 头文件 → include/
  ├─ 配置文件 → share/polygon_manager/config/
  └─ Launch 文件 → share/polygon_manager/launch/
```

## 依赖关系图

```
polygon_manager_node
    ├─── rclcpp
    ├─── polygon_manager_lib
    │     ├─── geometry_utils.cpp
    │     ├─── marker_utils.cpp
    │     └─── polygon_manager.cpp
    └─── ROS 消息库
          ├─── geometry_msgs
          ├─── visualization_msgs
          ├─── nav_msgs
          └─── std_msgs

bt_enemy_in_area
    ├─── behaviortree_cpp_v4
    ├─── polygon_manager_lib
    └─── geometry_msgs
```

## 代码统计

| 文件 | 行数 | 类型 | 说明 |
|------|------|------|------|
| polygon_area.hpp | 80 | 头文件 | 数据结构 |
| geometry_utils.hpp | 250 | 头文件 | 几何算法接口 |
| geometry_utils.cpp | 350 | 源文件 | 几何算法实现 |
| marker_utils.hpp | 200 | 头文件 | Marker 工具接口 |
| marker_utils.cpp | 200 | 源文件 | Marker 工具实现 |
| polygon_manager.hpp | 350 | 头文件 | 核心管理接口 |
| polygon_manager.cpp | 450 | 源文件 | 核心管理实现 |
| polygon_node.cpp | 250 | 源文件 | ROS2 节点 |
| bt_enemy_in_area.cpp | 300 | 源文件 | BehaviorTree 节点 |
| CMakeLists.txt | 80 | 构建 | 构建配置 |
| package.xml | 50 | 配置 | 包元数据 |
| README.md | 800 | 文档 | 完整文档 |
| QUICK_START.md | 200 | 文档 | 快速开始 |
| **总计** | **~4000** | | |

## 修改指南

### 添加新的区域类型

1. **修改 polygon_area.hpp**：
   ```cpp
   enum class AreaType {
     // ... 现有类型
     NEW_AREA = 4
   };
   ```

2. **修改 marker_utils.cpp**：
   ```cpp
   case AreaType::NEW_AREA:
     color.r = ...; color.g = ...; color.b = ...;
     break;
   ```

3. **修改 polygon_manager.cpp**：
   ```cpp
   // 在 stringToAreaType() 中添加转换
   if (type_str == "newtype") return AreaType::NEW_AREA;
   
   // 在 areaTypeToString() 中添加转换
   case AreaType::NEW_AREA: return "newtype";
   ```

### 修改 Marker 样式

编辑 `marker_utils.cpp` 中的常数或 `createMarkerFromPolygon()`：

```cpp
// 改变线宽
static constexpr float SCALE_X = 0.05f;  // 更粗

// 改变颜色透明度
marker.color.a = 0.5f;  // 半透明
```

### 优化几何算法

在 `geometry_utils.cpp` 中：

- 改变 `EPSILON` 值以调整浮点精度
- 实现空间索引（四叉树）以加速大规模多边形检测
- 添加多线程支持

## 下一步

- **学习源代码**：从 `README.md` 的"几何算法详解"开始
- **运行测试**：使用 `test_demo.py` 验证系统
- **扩展功能**：参考"修改指南"添加新功能
- **集成 BehaviorTree**：参考 `README.md` 的"BehaviorTree 集成"部分
