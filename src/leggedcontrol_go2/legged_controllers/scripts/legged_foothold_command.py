#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from legged_terrain_msgs.msg import FootholdPlan


FOOT_NAMES = ["LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"]


def parse_args():
    parser = argparse.ArgumentParser(description="Publish the first terrain-aware foothold height plan")
    parser.add_argument("--liftoff", type=float, default=0.0, help="current support height in odom [m]")
    parser.add_argument("--touchdown", type=float, default=0.08, help="next support height in odom [m]")
    parser.add_argument("--apex", type=float, default=0.15, help="minimum absolute swing apex in odom [m]")
    parser.add_argument("--forward", type=float, default=0.22, help="stored for inspection; this height-only command disables x/y tracking")
    parser.add_argument("--clear", action="store_true", help="clear every foothold override")
    return parser.parse_args()


class FootholdCommand(Node):
    def __init__(self, args):
        super().__init__("legged_foothold_command")
        self.publisher = self.create_publisher(FootholdPlan, "/legged_robot/foothold_plan", 1)
        self.message = FootholdPlan()
        self.message.header.frame_id = "odom"
        self.message.foot_names = FOOT_NAMES
        for index in range(4):
            self.message.lift_off_positions[index].z = args.liftoff
            self.message.touch_down_positions[index].x = args.forward
            self.message.touch_down_positions[index].z = args.touchdown
            self.message.minimum_apex_heights[index] = args.apex
            self.message.valid[index] = not args.clear
            self.message.horizontal_valid[index] = False

    def send(self):
        # Send a few copies so this one-shot command is robust to DDS discovery.
        for _ in range(5):
            self.message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.message)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)


def main():
    args = parse_args()
    rclpy.init(args=[])
    node = FootholdCommand(args)
    try:
        node.send()
        action = "cleared" if args.clear else "published"
        node.get_logger().info(
            f"{action} foothold plan: liftoff={args.liftoff:.3f}, "
            f"touchdown={args.touchdown:.3f}, apex={args.apex:.3f}"
        )
        if not args.clear:
            node.get_logger().info("Published a height-only plan; horizontal tracking remains disabled for this command.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
