#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SlamToolboxMapSaver(Node):
    def __init__(self):
        super().__init__('slamtoolbox_map_saver')

        # ============================================================
        # 1. 触发话题配置
        # ============================================================

        # 保存触发话题
        # 类型：std_msgs/msg/Bool
        # 当 data == true 时保存地图
        self.trigger_topic = '/start_save_map'

        # ============================================================
        # 2. 地图保存路径配置
        # ============================================================

        # 地图保存目录
        self.output_dir = '/home/sentry/Desktop/ros_ws/map'

        # 地图文件名前缀
        # 最终会生成：
        # slam_map_20260524_153012.yaml
        # slam_map_20260524_153012.pgm
        self.map_name_prefix = 'slam_map'

        # ============================================================
        # 3. 状态变量
        # ============================================================

        # 防止重复保存
        self.saving = False

        # 保存完成后退出
        self.should_exit = False

        # ============================================================
        # 4. ROS2 订阅
        # ============================================================

        self.trigger_sub = self.create_subscription(
            Bool,
            self.trigger_topic,
            self.trigger_callback,
            10
        )

        self.get_logger().info('slamtoolbox_map_saver 已启动')
        self.get_logger().info(f'等待触发话题: {self.trigger_topic}')
        self.get_logger().info(f'地图保存目录: {self.output_dir}')
        self.get_logger().info('收到 /start_save_map=true 后，将保存 slam_toolbox 地图并退出')

    def trigger_callback(self, msg: Bool):
        """
        /start_save_map 触发回调。

        当收到 true 后：
        1. 防止重复触发
        2. 执行 map_saver_cli
        3. 保存完成后退出节点
        """

        if not msg.data:
            return

        if self.saving:
            return

        self.saving = True

        self.get_logger().info('收到 /start_save_map=true，开始保存 slam_toolbox 地图')

        self.save_map()

        self.get_logger().info('保存流程结束，节点即将退出')

        self.should_exit = True

    def save_map(self):
        """
        调用 nav2_map_server 的 map_saver_cli 保存地图。

        实际执行命令类似：

        ros2 run nav2_map_server map_saver_cli -f /home/sentry/Desktop/ros_ws/slam_maps/slam_map_20260524_153012

        注意：
        - -f 后面不要写 .yaml
        - map_saver_cli 会自动生成 .yaml 和 .pgm
        """

        os.makedirs(self.output_dir, exist_ok=True)

        time_str = datetime.now().strftime('%Y%m%d_%H%M%S')

        map_filename = f'{self.map_name_prefix}_{time_str}'

        output_path = os.path.join(self.output_dir, map_filename)

        cmd = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            output_path,
            '--ros-args',
            '-p', 'save_map_timeout:=10.0',
            '-p', 'map_subscribe_transient_local:=true',
            '-p', 'free_thresh:=0.25',
            '-p', 'occupied_thresh:=0.65'
        ]

        self.get_logger().info('执行保存地图命令:')
        self.get_logger().info(' '.join(cmd))

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.stdout:
                self.get_logger().info(result.stdout)

            if result.stderr:
                self.get_logger().warn(result.stderr)

            if result.returncode == 0:
                self.get_logger().info(f'地图保存成功: {output_path}.yaml / {output_path}.pgm')
            else:
                self.get_logger().error(f'地图保存失败，返回码: {result.returncode}')

        except subprocess.TimeoutExpired:
            self.get_logger().error('地图保存超时，map_saver_cli 可能没有收到 /map')

        except Exception as e:
            self.get_logger().error(f'执行保存地图命令异常: {e}')


def main():
    rclpy.init()

    node = SlamToolboxMapSaver()

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warn('收到 Ctrl+C，程序退出，不自动保存地图')

    finally:
        node.get_logger().info('正在销毁节点并关闭 rclpy')

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
