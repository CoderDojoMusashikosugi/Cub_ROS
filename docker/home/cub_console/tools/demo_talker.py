#!/usr/bin/env python3
"""プロセス操作の動作確認用の最小 talker(demo_nodes_cpp の talker 相当)。"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self) -> None:
        super().__init__("demo_talker")
        self._pub = self.create_publisher(String, "chatter", 10)
        self._count = 0
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        self._count += 1
        msg = String(data=f"Hello World: {self._count}")
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
