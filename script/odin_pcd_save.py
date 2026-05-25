#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np
import open3d as o3d


class OdinCloudToPCD(Node):
    def __init__(self):
        super().__init__('odin1_cloud_to_pcd')

        # ============================================================
        # 1. 话题配置
        # ============================================================

        # Odin1 发布的点云话题
        # 你说现在 odin1/cloud_slam 是 XYZI，且 intensity = 0
        self.cloud_topic = '/odin1/cloud_slam'

        # 保存触发话题
        # 类型：std_msgs/msg/Bool
        # 当 data == true 时保存 PCD
        self.trigger_topic = '/start_save_map'

        # ============================================================
        # 2. 保存路径配置
        # ============================================================

        # PCD 保存目录
        # 最终文件名会自动按时间生成，例如：
        # odin1_map_20260520_153012.pcd
        self.output_dir = '/home/sentry/Desktop/ros_ws/pcd_map'

        # ============================================================
        # 3. 点云过滤与降采样参数
        # ============================================================

        # 单帧体素降采样，单位 m
        self.frame_voxel_size = 0.03

        # 全局体素降采样，单位 m
        self.global_voxel_size = 0.05

        # 高度过滤范围
        # 只保留 min_z <= z <= max_z 的点
        #
        # 如果 /odin1/cloud_slam 已经是 map/odom 坐标系，
        # 那这里的 z 就是世界坐标系下的高度。
        #
        # 你主要说要过滤过高点，所以 min_z 默认放得很低。
        self.min_z = -10.0
        self.max_z = 2.0

        # 全局点数超过这个数量时，强制全局降采样
        self.max_global_points_before_downsample = 800000

        # 每多少帧做一次周期性全局降采样
        self.global_downsample_every_n_frames = 50

        # 每多少帧打印一次状态
        self.log_every_n_frames = 30

        # PCD 是否压缩保存
        self.pcd_compressed = True

        # 保存完成后是否自动退出
        self.exit_after_save = True

        # ============================================================
        # 4. 状态变量
        # ============================================================

        # 全局点云
        # 这里存的是已经处理过的点云，不是原始点云
        self.global_cloud = o3d.geometry.PointCloud()

        # 已接收点云帧数
        self.frame_count = 0

        # 是否正在保存
        # 保存期间不再处理新的点云
        self.saving = False

        # 是否保存完成并准备退出
        self.should_exit = False

        # 记录启动时间
        self.start_time = time.time()

        # ============================================================
        # 5. ROS2 订阅
        # ============================================================

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            10
        )

        self.trigger_sub = self.create_subscription(
            Bool,
            self.trigger_topic,
            self.trigger_callback,
            10
        )

        self.get_logger().info('odin1_cloud_to_pcd 已启动')
        self.get_logger().info(f'接收点云话题: {self.cloud_topic}')
        self.get_logger().info(f'保存触发话题: {self.trigger_topic}')
        self.get_logger().info(f'PCD 保存目录: {self.output_dir}')
        self.get_logger().info(f'单帧降采样: {self.frame_voxel_size} m')
        self.get_logger().info(f'全局降采样: {self.global_voxel_size} m')
        self.get_logger().info(f'高度过滤: {self.min_z} <= z <= {self.max_z}')
        self.get_logger().info(f'PCD 压缩保存: {self.pcd_compressed}')
        self.get_logger().info('收到 /start_save_map=true 后，将保存 PCD 并退出')

    def cloud_callback(self, msg: PointCloud2):
        """
        点云回调函数。

        每收到一帧 PointCloud2：
        1. 读取 x y z
        2. 过滤 NaN
        3. 过滤高度
        4. 单帧降采样
        5. 累积到全局点云
        6. 周期性全局降采样
        """

        # 如果已经触发保存，就不要再处理新点云
        if self.saving:
            return

        points = []

        # 从 PointCloud2 中读取 x y z
        # 虽然你的点云是 XYZI，但是 intensity = 0，
        # 所以这里暂时只保存 xyz。
        for p in pc2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        ):
            x = float(p[0])
            y = float(p[1])
            z = float(p[2])

            # 高度过滤
            if z < self.min_z or z > self.max_z:
                continue

            points.append([x, y, z])

        # 当前帧没有有效点，直接跳过
        if len(points) == 0:
            return

        # 转成 numpy
        points_np = np.asarray(points, dtype=np.float64)

        # 构造当前帧 Open3D 点云
        frame_cloud = o3d.geometry.PointCloud()
        frame_cloud.points = o3d.utility.Vector3dVector(points_np)

        # 单帧降采样
        frame_cloud = frame_cloud.voxel_down_sample(
            voxel_size=self.frame_voxel_size
        )

        # 累积到全局点云
        self.global_cloud += frame_cloud

        self.frame_count += 1

        # 如果全局点云数量太大，做一次全局降采样
        global_points_num = len(self.global_cloud.points)

        # 点数过大时强制全局降采样
        if global_points_num > self.max_global_points_before_downsample:
            self.downsample_global_cloud(
                reason=f'全局点数超过阈值 {self.max_global_points_before_downsample}'
            )

        # 周期性全局降采样
        if self.frame_count % self.global_downsample_every_n_frames == 0:
            self.downsample_global_cloud(
                reason=f'周期性降采样，每 {self.global_downsample_every_n_frames} 帧'
            )

        # 定期打印状态
        if self.frame_count % self.log_every_n_frames == 0:
            elapsed = time.time() - self.start_time

            self.get_logger().info(
                f'已接收 {self.frame_count} 帧，'
                f'当前全局点数: {len(self.global_cloud.points)}，'
                f'运行时间: {elapsed:.1f}s'
            )

    def downsample_global_cloud(self, reason: str = ''):
        """
        对全局点云做体素降采样。

        作用：
        - 减少重复点
        - 降低内存
        - 缩短最终保存时间
        """

        before_num = len(self.global_cloud.points)

        if before_num == 0:
            return

        if reason:
            self.get_logger().info(f'开始全局降采样，原因: {reason}')
        else:
            self.get_logger().info('开始全局降采样')

        self.global_cloud = self.global_cloud.voxel_down_sample(
            voxel_size=self.global_voxel_size
        )

        after_num = len(self.global_cloud.points)

        self.get_logger().info(
            f'全局降采样完成: {before_num} -> {after_num}'
        )

    def trigger_callback(self, msg: Bool):
        """
        /start_save_map 触发回调。

        data == true 时：
        1. 停止处理新点云
        2. 保存 PCD
        3. 检查文件
        4. sync 刷盘
        5. 退出节点
        """

        # 只响应 true
        if not msg.data:
            return

        # 防止重复触发
        if self.saving:
            self.get_logger().warn('正在保存 PCD，忽略重复触发')
            return

        self.saving = True

        self.get_logger().info('收到 /start_save_map=true，开始保存 PCD')

        success = self.save_pcd()

        if success:
            self.get_logger().info('PCD 保存流程成功结束')
        else:
            self.get_logger().error('PCD 保存流程失败')

        if self.exit_after_save:
            self.should_exit = True

    def wait_file_ready(self, file_path: str, timeout_sec: float = 10.0) -> bool:
        """
        等待文件真正生成，并且大小大于 0。

        防止：
        - 文件刚创建但还没写入
        - 文件存在但是 0KB
        """

        start_time = time.time()

        while time.time() - start_time < timeout_sec:

            if os.path.exists(file_path):
                size = os.path.getsize(file_path)

                if size > 0:
                    self.get_logger().info(
                        f'文件已生成: {file_path}, size={size} bytes'
                    )
                    return True

                self.get_logger().warn(
                    f'文件存在但大小为 0，继续等待: {file_path}'
                )

            time.sleep(0.2)

        return False

    def save_pcd(self) -> bool:
        """
        保存 PCD 文件。

        保存流程：
        1. 检查当前全局点云是否为空
        2. 保存前最后全局降采样
        3. 写入 PCD
        4. 检查文件大小
        5. sync 强制刷盘
        6. 延迟等待
        """
        # 是否保存成功后自动关机
        self.auto_poweroff = True

# 保存成功后延迟几秒再关机
        self.poweroff_delay_sec = 20.0

        point_num = len(self.global_cloud.points)

        if point_num == 0:
            self.get_logger().error('当前没有任何有效点云，无法保存 PCD')
            return False

        self.get_logger().info(f'保存前全局点数: {point_num}')

        # 保存前最后再全局降采样一次
        final_cloud = self.global_cloud.voxel_down_sample(
            voxel_size=self.global_voxel_size
        )

        final_point_num = len(final_cloud.points)

        if final_point_num == 0:
            self.get_logger().error('最终降采样后点数为 0，无法保存 PCD')
            return False

        self.get_logger().info(f'最终降采样后点数: {final_point_num}')

        # 创建保存目录
        os.makedirs(self.output_dir, exist_ok=True)

        # 当前时间字符串
        time_str = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 自动生成文件名
        filename = f'odin1_map_{time_str}.pcd'

        # 拼接完整路径
        output_path = os.path.join(self.output_dir, filename)

        self.get_logger().info(f'开始保存 PCD: {output_path}')

        ok = o3d.io.write_point_cloud(
            output_path,
            final_cloud,
            write_ascii=False,
            compressed=self.pcd_compressed
        )

        if not ok:
            self.get_logger().error('Open3D write_point_cloud 返回失败')
            return False

        self.get_logger().info('Open3D 写入完成，开始检查文件')

        file_ok = self.wait_file_ready(
            output_path,
            timeout_sec=10.0
        )

        if not file_ok:
            if os.path.exists(output_path):
                size = os.path.getsize(output_path)
                self.get_logger().error(f'PCD 文件异常，当前大小: {size} bytes')
            else:
                self.get_logger().error(f'PCD 文件不存在: {output_path}')

            return False

        try:
            size_before_sync = os.path.getsize(output_path)
            self.get_logger().info(
                f'sync 前文件大小: {size_before_sync / 1024 / 1024:.2f} MB'
            )
        except Exception as e:
            self.get_logger().warn(f'读取 sync 前文件大小失败: {e}')

        self.get_logger().info('开始执行 sync，强制刷盘')

        try:
            sync_result = subprocess.run(
                ['sync'],
                capture_output=True,
                text=True,
                timeout=20
            )

            if sync_result.returncode != 0:
                self.get_logger().error(
                    f'sync 执行失败，返回码: {sync_result.returncode}'
                )

                if sync_result.stderr:
                    self.get_logger().error(sync_result.stderr)

                return False

        except subprocess.TimeoutExpired:
            self.get_logger().error('sync 超时，磁盘可能写入过慢')
            return False

        except Exception as e:
            self.get_logger().error(f'sync 执行异常: {e}')
            return False

        self.get_logger().info('sync 完成，PCD 文件已尽量写入磁盘')

        # 再等待几秒，避免刚 sync 完马上断电
        time.sleep(3.0)

        try:
            final_size = os.path.getsize(output_path)
            self.get_logger().info(
                f'最终文件大小: {final_size / 1024 / 1024:.2f} MB'
            )
        except Exception as e:
            self.get_logger().warn(f'读取最终文件大小失败: {e}')

        self.get_logger().info(f'PCD 保存成功: {output_path}')
        self.get_logger().info('现在可以安全关机')
        if self.auto_poweroff:
            self.get_logger().warn(
            f'{self.poweroff_delay_sec} 秒后自动关机'
        )

        time.sleep(self.poweroff_delay_sec)

        try:
            subprocess.run(
                ['sudo', 'systemctl', 'poweroff'],
                capture_output=True,
                text=True,
                timeout=10
            )
        except Exception as e:
            self.get_logger().error(f'自动关机失败: {e}')
            return False
        else:
            self.get_logger().info('现在可以安全关机')

        return True


def main():
    rclpy.init()

    node = OdinCloudToPCD()

    try:
        # 不使用 rclpy.spin(node)
        # 因为 rclpy.spin(node) 不方便在保存后主动退出
        #
        # 这里使用 spin_once：
        # 每次处理一点 ROS 回调，然后检查 node.should_exit
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().warn('收到 Ctrl+C，程序退出，不自动保存 PCD')

    finally:
        node.get_logger().info('正在销毁节点并关闭 rclpy')

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
