#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GaitCommandCli(Node):
    def __init__(self):
        super().__init__("legged_gait_command_cli")
        self.declare_parameter("command_topic", "/legged_robot_gait_command")
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(String, command_topic, 10)
        self.get_logger().info(f"publishing gait commands to {command_topic}")

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitCommandCli()
    try:
        print("Type a gait name, 'list', or 'quit'.")
        while rclpy.ok():
            try:
                command = input("gait> ").strip()
            except EOFError:
                break
            if command.lower() in ("quit", "exit"):
                break
            if command:
                node.publish_command(command)
                rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
