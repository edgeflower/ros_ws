#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import time
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
            self.get_logger().warn('正在保存地图，忽略重复触发')
            return

        self.saving = True

        self.get_logger().info('收到 /start_save_map=true，开始保存 slam_toolbox 地图')

        success = self.save_map()

        if success:
            self.get_logger().info('保存流程成功结束，节点即将退出')
        else:
            self.get_logger().error('保存流程失败，节点即将退出')

        self.should_exit = True

    def wait_file_ready(self, file_path: str, timeout_sec: float = 5.0) -> bool:
        """
        等待文件真正生成，并且大小大于 0。

        主要用于防止：
        - yaml 已生成
        - pgm 文件存在
        - 但是 pgm 还是 0KB
        """

        start_time = time.time()

        while time.time() - start_time < timeout_sec:
            if os.path.exists(file_path):
                size = os.path.getsize(file_path)

                if size > 0:
                    self.get_logger().info(f'文件已就绪: {file_path}, size={size} bytes')
                    return True

                self.get_logger().warn(f'文件存在但大小为 0，继续等待: {file_path}')

            time.sleep(0.2)

        return False

    def save_map(self) -> bool:
        os.makedirs(self.output_dir, exist_ok=True)

        time_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_filename = f'{self.map_name_prefix}_{time_str}'
        output_path = os.path.join(self.output_dir, map_filename)

        yaml_path = output_path + '.yaml'
        pgm_path = output_path + '.pgm'

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

            if result.returncode != 0:
                self.get_logger().error(f'map_saver_cli 执行失败，返回码: {result.returncode}')
                return False

            self.get_logger().info('map_saver_cli 执行完成，开始检查地图文件')

            yaml_ok = os.path.exists(yaml_path)
            pgm_ok = self.wait_file_ready(pgm_path, timeout_sec=5.0)

            if not yaml_ok:
                self.get_logger().error(f'yaml 文件不存在: {yaml_path}')
                return False

            if not pgm_ok:
                if os.path.exists(pgm_path):
                    size = os.path.getsize(pgm_path)
                    self.get_logger().error(f'pgm 文件异常，当前大小: {size} bytes')
                else:
                    self.get_logger().error(f'pgm 文件不存在: {pgm_path}')
                return False

            self.get_logger().info('地图文件检查通过')
            self.get_logger().info(f'yaml: {yaml_path}')
            self.get_logger().info(f'pgm : {pgm_path}')

            self.get_logger().info('开始执行 sync，强制刷盘')

            sync_result = subprocess.run(
                ['sync'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if sync_result.returncode != 0:
                self.get_logger().error(f'sync 执行失败，返回码: {sync_result.returncode}')
                return False

            self.get_logger().info('sync 完成，地图文件已尽量写入磁盘')

            time.sleep(2.0)

            self.get_logger().info('地图保存成功，可以安全关机')
            return True

        except subprocess.TimeoutExpired:
            self.get_logger().error('地图保存超时，map_saver_cli 可能没有收到 /map')
            return False

        except Exception as e:
            self.get_logger().error(f'执行保存地图命令异常: {e}')
            return False


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
