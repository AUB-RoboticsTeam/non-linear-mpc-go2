#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gait_command_common import GaitCommandFile


class GaitCommandBridge(Node):
    def __init__(self):
        super().__init__("legged_gait_command_bridge")
        self.declare_parameter("gaitCommandFile", "")
        self.declare_parameter("robotName", "legged_robot")
        self.declare_parameter("command_topic", "/legged_robot_gait_command")

        gait_file = self.get_parameter("gaitCommandFile").get_parameter_value().string_value
        robot_name = self.get_parameter("robotName").get_parameter_value().string_value
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        if not gait_file:
            raise RuntimeError("parameter 'gaitCommandFile' is required")

        self.gaits = GaitCommandFile(gait_file)
        self.publisher = self.create_publisher(
            self._mode_schedule_type(), f"{robot_name}_mpc_mode_schedule", 1
        )
        self.subscription = self.create_subscription(String, command_topic, self._command_callback, 10)

        self.get_logger().info(
            f"gait command bridge ready on {command_topic}; available: {', '.join(self.gaits.gait_names)}"
        )

    @staticmethod
    def _mode_schedule_type():
        from ocs2_msgs.msg import ModeSchedule

        return ModeSchedule

    def _command_callback(self, msg: String):
        command = msg.data.strip().lower()
        if not command:
            return
        if command == "list":
            self.get_logger().info("available gaits: " + ", ".join(self.gaits.gait_names))
            return
        try:
            schedule = self.gaits.create_message(command)
        except (KeyError, ValueError) as exc:
            self.get_logger().warn(f"{exc}; available: {', '.join(self.gaits.gait_names)}")
            return

        self.publisher.publish(schedule)
        self.get_logger().info(f"published gait '{command}'")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GaitCommandBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"legged_gait_command_bridge: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
