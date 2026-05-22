# rm_keepout_costmap_layer

NAV2 Costmap Layer 插件，通过 YAML 配置文件定义多边形禁行区域，将其标记为障碍物写入代价地图。

## 功能特性

- 从 YAML 文件加载多边形禁行区域，填充到 costmap 中
- 热加载：定时检测文件修改时间，自动重新加载，无需重启节点
- 运行时动态参数调整（`enabled`、`keepout_cost`）
- 大区域告警：bounding box 超过阈值时输出警告
- 基于 ray casting 的点在多边形内判断

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | bool | `true` | 是否启用该层 |
| `yaml_path` | string | `/home/sentry/areas.yaml` | 禁行区域 YAML 文件路径 |
| `keepout_cost` | int | `254` | 禁行区域代价值，范围 [0, 254] |
| `reload_interval_sec` | double | `2.0` | YAML 文件变更检测间隔（秒） |
| `areas_frame` | string | `map` | 多边形坐标所在坐标系（必须与 costmap 全局坐标系一致） |
| `huge_area_warn_threshold` | double | `20.0` | 大区域告警阈值（bounding box 面积，m²） |

## YAML 配置格式

```yaml
areas:
  - name: enemy_danger
    type: forbidden
    points:
      - [1.0, 1.0]
      - [3.0, 1.0]
      - [3.0, 3.0]
      - [1.0, 3.0]
```

- `name`：区域名称（可选，默认 `area_N`）
- `type`：**仅 `forbidden` 类型会被处理**
- `points`：多边形顶点列表 `[x, y]`，至少 3 个点

## 在 NAV2 中使用

在 `nav2_params.yaml` 的 costmap 配置中添加插件：

```yaml
local_costmap:
  ros__parameters:
    plugins:
      - static_layer
      - obstacle_layer
      - inflation_layer
      - polygon_keepout_layer          # 添加此行
   polygon_keepout_layer:
      plugin: rm_keepout_costmap_layer::PolygonKeepoutLayer
      enabled: true
      yaml_path: /home/sentry/areas.yaml
      keepout_cost: 254
      areas_frame: map
```

## 编译

```bash
colcon build --packages-select rm_keepout_costmap_layer
```

## 依赖

- ROS2 Humble
- nav2_costmap_2d
- yaml-cpp



## 注意

**此layer推荐只在global_costmap中使用，frame为 map**；

在local_costmap中使用 frame为odom，由于重定位时odom与map不重叠，而此layer的areas.yaml是在map中配置的，所以会有偏差，不推荐使用。
