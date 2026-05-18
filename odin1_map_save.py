#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SaveMapTriggerNode(Node):
    def __init__(self):
        super().__init__("save_map_trigger_node")

        self.last_state = False

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
                    ["./set_param.sh", "save_map", "1"],
                    cwd="/home/sentry/Desktop/ros_ws/src/odin_ros_driver",
                    capture_output=True,
                    text=True
                )

                self.get_logger().info(f"returncode: {result.returncode}")

                if result.stdout:
                    self.get_logger().info(f"stdout:\n{result.stdout}")

                if result.stderr:
                    self.get_logger().warn(f"stderr:\n{result.stderr}")

                if result.returncode == 0:
                    self.get_logger().info("保存地图命令执行成功")
                else:
                    self.get_logger().error("保存地图命令执行失败")

            except Exception as e:
                self.get_logger().error(f"执行保存地图命令异常: {e}")

        self.last_state = current_state


def main(args=None):
    rclpy.init(args=args)

    node = SaveMapTriggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
