# 快速开始指南

## 5 分钟内让系统运行起来

### 第一步：编译

```bash
# 假设已经有 ROS2 Humble 工作空间
cd ~/ros2_ws

# 复制包到 src 目录
cp -r polygon_manager src/

# 安装依赖
rosdep install --from-paths src --ignore-src -y

# 编译
colcon build --packages-select polygon_manager

# 加载环境
source install/setup.bash
```

### 第二步：启动系统

**终端 1** - 启动 Polygon Manager 节点：

```bash
ros2 launch polygon_manager polygon_manager.launch.py
```

你应该看到类似的输出：

```
[polygon_manager_node-1] [INFO] RoboMaster Polygon Manager Node Starting...
[polygon_manager_node-1] [INFO] Successfully loaded config from: config/areas.yaml
[polygon_manager_node-1] [INFO] Polygon Manager Node Ready!
[rviz2-2] [INFO] Starting RViz2...
```

**RViz 会自动打开**，显示地图和已加载的多边形区域。

### 第三步：测试敌方检测

**终端 2** - 发送敌方位置：

```bash
# 发送一个敌方在禁区的位置
ros2 topic pub /enemy_position geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.5, y: 2.5, z: 0}, orientation: {w: 1.0}}}'
```

**终端 3** - 查看检测结果：

```bash
ros2 topic echo /enemy_area_state
```

你应该看到：

```
data: enemy_in_forbidden_zone_1
```

## 常用命令速查

### 查看所有 Topic

```bash
ros2 topic list
```

### 查看 Polygon Manager 发布的数据

```bash
# 敌方状态
ros2 topic echo /enemy_area_state

# Marker 可视化数据
ros2 topic echo /polygon_areas/visualization_marker_array
```

### 查看地图信息

```bash
ros2 topic info /map
```

### 保存配置

配置在每次添加新多边形后会自动保存到 `config/areas.yaml`。

要手动保存：

```bash
# 配置文件自动保存，无需手动操作
```

## 在 RViz 中交互式创建多边形

1. **RViz 启动后**，在工具栏找到 "Publish Point" 工具（紫色图标）

2. **点击选中该工具**，光标会变成十字

3. **在地图上依次点击**多边形的各个顶点：
   - 第 1 个点：系统自动开始创建新多边形
   - 第 2、3、4 个点：继续添加顶点
   - 第 4 个点后：系统自动闭合多边形并保存

4. **在 RViz 窗口左侧 Displays 面板**，可以看到：
   - 已定义的多边形以相应颜色显示
   - 当前正在创建的多边形以白色显示

## 配置文件编辑

编辑 `config/areas.yaml` 来预定义区域：

```yaml
areas:
  # 禁区（红色）
  - name: forbidden_zone_1
    type: forbidden
    points:
      - [1.0, 1.0]
      - [5.0, 1.0]
      - [5.0, 4.0]
      - [1.0, 4.0]

  # 防守区（绿色）
  - name: protect_zone_1
    type: protect
    points:
      - [7.0, 2.0]
      - [9.0, 2.0]
      - [9.0, 5.0]
      - [7.0, 5.0]
```

修改后重新启动节点以加载新配置。

## 常见问题快速解决

### Q: RViz 中看不到多边形

**A:** 在 RViz 左侧 Displays 面板中：
1. 找到 "Polygon Areas" 显示
2. 勾选其前面的复选框以启用

### Q: 点击工具不发布点

**A:** 确保：
1. 在 RViz 中选中了 "Publish Point" 工具
2. Topic 设置为 `/clicked_point`
3. Frame 设置为 `map`

### Q: 敌方位置发布但没有反应

**A:** 检查：
1. 敌方位置的 frame_id 是否为 "map"
2. 位置坐标是否在多边形范围内
3. 使用 `ros2 topic echo` 验证数据

### Q: 编译失败

**A:** 运行：

```bash
rosdep install --from-paths src --ignore-src -y
rm -rf build install
colcon build --packages-select polygon_manager
```

## 性能测试

测试系统性能：

```bash
# 启动节点（记下启动时间）
ros2 launch polygon_manager polygon_manager.launch.py

# 在另一个终端，持续发送敌方位置
for i in {1..100}; do
  x=$((RANDOM % 10))
  y=$((RANDOM % 10))
  ros2 topic pub /enemy_position geometry_msgs/PoseStamped \
    --once \
    "{header: {frame_id: \"map\"}, pose: {position: {x: $x, y: $y, z: 0}, orientation: {w: 1.0}}}"
done

# 观察日志输出，检查处理延迟
```

## 下一步

- 详细文档：查看 `README.md`
- 源代码注释：查看 `.hpp` 和 `.cpp` 文件
- 添加更多区域类型：编辑 `polygon_area.hpp`
- 集成 BehaviorTree：查看 `bt_enemy_in_area.cpp`

## 常用参数

启动时可以指定参数：

```bash
ros2 launch polygon_manager polygon_manager.launch.py \
  config_file:=/custom/path/areas.yaml \
  update_frequency:=20.0 \
  use_rviz:=true
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `config_file` | `config/areas.yaml` | 配置文件路径 |
| `update_frequency` | `10.0` | Marker 发布频率 (Hz) |
| `use_rviz` | `true` | 是否启动 RViz |

---

**提示**：如果卡在某个步骤，查看完整的 `README.md` 文档获得更详细的说明。
