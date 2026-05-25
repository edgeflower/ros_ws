#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SaveMapTriggerNode(Node):
    def __init__(self):
        super().__init__("save_map_trigger_node")

        # 上一次 /start_save_map 的状态
        # 用于检测 false -> true 的上升沿
        self.last_state = False

        # 命令执行成功后设置为 True
        # main() 检测到后会退出节点
        self.should_exit = False

        self.sub = self.create_subscription(
            Bool,
            "/start_save_map",
            self.callback,
            10
        )

        self.get_logger().info("save_map_trigger_node 已启动，等待 /start_save_map")

    def callback(self, msg: Bool):
        current_state = msg.data

        # 只在 false -> true 的上升沿触发一次
        if current_state and not self.last_state:
            self.get_logger().info("收到 start_save_map=true，开始执行保存地图命令")

            try:
                result = subprocess.run(
                "source /opt/ros/humble/setup.bash && "
                "source /home/sentry/Desktop/ros_ws/install/setup.bash && "
                "cd /home/sentry/Desktop/ros_ws/src/odin_ros_driver && "
                "./set_param.sh save_map 1",
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True
                )

                self.get_logger().info(f"returncode: {result.returncode}")

                if result.stdout:
                    self.get_logger().info(f"stdout:\n{result.stdout}")

                if result.stderr:
                    self.get_logger().warn(f"stderr:\n{result.stderr}")

                if result.returncode == 0:
                    self.get_logger().info("保存地图命令执行成功，节点即将退出")

                    # 执行成功后退出
                    self.should_exit = True
                else:
                    self.get_logger().error("保存地图命令执行失败，节点继续等待下一次触发")

            except Exception as e:
                self.get_logger().error(f"执行保存地图命令异常: {e}")

        self.last_state = current_state


def main(args=None):
    rclpy.init(args=args)

    node = SaveMapTriggerNode()

    try:
        # 不使用 rclpy.spin(node)
        # 因为 spin 会一直阻塞，不方便在命令成功后主动退出
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，节点退出")

    finally:
        node.get_logger().info("正在销毁节点并关闭 rclpy")
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
