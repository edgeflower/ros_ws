#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
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

        # 单帧体素降采样尺寸，单位 m
        # 每收到一帧点云，先对这一帧做一次降采样
        self.frame_voxel_size = 0.03

        # 全局地图体素降采样尺寸，单位 m
        # 多帧点云累积后，对全局点云再降采样
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

        # 全局点云超过这个数量时，自动做一次全局降采样
        # 防止内存一直增长
        self.max_global_points_before_downsample = 800000

        # 每接收多少帧打印一次状态
        self.log_every_n_frames = 30

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
        self.get_logger().info('正在一边接收点云，一边过滤、降采样、累积')
        self.get_logger().info('收到 /start_save_map=true 后，将保存 PCD 并退出')

    def cloud_callback(self, msg: PointCloud2):
        """
        点云回调函数。

        每收到一帧 PointCloud2：
        1. 读取 x y z
        2. 过滤 NaN
        3. 过滤过高点
        4. 对当前帧做体素降采样
        5. 加入全局点云
        6. 全局点云太大时，再整体降采样
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

        if global_points_num > self.max_global_points_before_downsample:
            self.get_logger().info(
                f'全局点数达到 {global_points_num}，开始全局降采样'
            )

            self.global_cloud = self.global_cloud.voxel_down_sample(
                voxel_size=self.global_voxel_size
            )

            self.get_logger().info(
                f'全局降采样后点数: {len(self.global_cloud.points)}'
            )

        # 定期打印当前状态
        if self.frame_count % self.log_every_n_frames == 0:
            elapsed = time.time() - self.start_time

            self.get_logger().info(
                f'已接收 {self.frame_count} 帧，'
                f'当前全局点数: {len(self.global_cloud.points)}，'
                f'运行时间: {elapsed:.1f}s'
            )

    def trigger_callback(self, msg: Bool):
        """
        /start_save_map 触发回调。

        当收到 true 后：
        1. 停止继续处理新点云
        2. 保存当前全局点云
        3. 设置退出标志
        """

        # 只响应 true
        if not msg.data:
            return

        # 防止重复触发
        if self.saving:
            return

        self.saving = True

        self.get_logger().info('收到 /start_save_map=true，开始保存 PCD')

        self.save_pcd()

        self.get_logger().info('保存流程结束，节点即将退出')

        self.should_exit = True

    def save_pcd(self):
        """
        保存 PCD 文件。

        文件名按照当前系统时间自动生成，例如：
        odin1_map_20260520_153012.pcd
        """

        point_num = len(self.global_cloud.points)

        if point_num == 0:
            self.get_logger().error('当前没有任何有效点云，无法保存 PCD')
            return

        self.get_logger().info(f'保存前全局点数: {point_num}')

        # 保存前最后再做一次全局降采样
        # 这样可以去掉不同帧之间重复累积的点
        final_cloud = self.global_cloud.voxel_down_sample(
            voxel_size=self.global_voxel_size
        )

        final_point_num = len(final_cloud.points)

        self.get_logger().info(f'最终降采样后点数: {final_point_num}')

        # 创建保存目录
        os.makedirs(self.output_dir, exist_ok=True)

        # 当前时间字符串
        time_str = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 自动生成文件名
        filename = f'odin1_map_{time_str}.pcd'

        # 拼接完整路径
        output_path = os.path.join(self.output_dir, filename)

        # 保存 PCD
        ok = o3d.io.write_point_cloud(
            output_path,
            final_cloud,
            write_ascii=False,
            compressed=False
        )

        if ok:
            self.get_logger().info(f'PCD 保存成功: {output_path}')
        else:
            self.get_logger().error(f'PCD 保存失败: {output_path}')


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
