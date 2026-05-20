#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist

import tf2_ros

# 你的比赛状态消息
from rm_decision_interfaces.msg import GameStatus


class TfMonitor(Node):

    def __init__(self):
        super().__init__('tf_monitor')

        # 当前比赛阶段
        self.game_progress = 0

        # cmd_vel 发布器
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_chassis',
            10
        )

        # 订阅 game_status
        self.game_status_sub = self.create_subscription(
            GameStatus,
            '/game_status',
            self.game_status_callback,
            10
        )

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()

        # TF Listener
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # 定时器
        self.timer = self.create_timer(
            0.2,
            self.timer_callback
        )

        self.get_logger().info(
            'TF Monitor Started'
        )

    def game_status_callback(self, msg):

        # 保存比赛阶段
        self.game_progress = msg.game_progress

    def timer_callback(self):

        # 只有比赛进行中(game_progress == 4)
        # 才允许发送速度
        if self.game_progress != 4:

            self.get_logger().info(
                f'Waiting game start, game_progress={self.game_progress}'
            )

            self.stop_robot()

            return

        try:

            # 查询 map -> odom
            self.tf_buffer.lookup_transform(
                'map',
                'odom',
                Time()
            )

            # 查询成功
            self.get_logger().info(
                'map -> odom tf found!'
            )

            # 停止机器人
            self.stop_robot()

            # 关闭程序
            rclpy.shutdown()

        except Exception as e:

            # 查询失败
            self.get_logger().warn(
                f'TF not found: {str(e)}'
            )

            # 发送速度
            self.send_cmd_vel()

    def send_cmd_vel(self):

        msg = Twist()

        # 原地旋转
        msg.angular.z = 0.0
        msg.linear.x = 0.5
        msg.linear.y = -0.5

        self.cmd_pub.publish(msg)

    def stop_robot(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0

        self.cmd_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = TfMonitor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
