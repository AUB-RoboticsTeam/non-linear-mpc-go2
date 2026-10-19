#!/usr/bin/env python3
"""Fail-fast validation of the paper-compatible terrain perception topics."""

import argparse
import math
import sys
import time

import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node

from legged_terrain_msgs.msg import PlanarTerrain


REQUIRED_LAYERS = {
    "elevation_inpainted", "elevation_median", "roughness", "steppability",
    "planar", "plane_id", "paper_normal_x", "paper_normal_y",
    "paper_normal_z", "torso_reference",
}


class Checker(Node):
    def __init__(self):
        super().__init__("check_perception_pipeline")
        self.map_times = []
        self.latest_map = None
        self.latest_terrain = None
        self.create_subscription(
            GridMap, "/perceptive_terrain_map", self.map_callback, 10)
        self.create_subscription(
            PlanarTerrain, "/planar_terrain", self.terrain_callback, 10)

    def map_callback(self, msg):
        self.map_times.append(time.monotonic())
        self.latest_map = msg

    def terrain_callback(self, msg):
        self.latest_terrain = msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--minimum-rate", type=float, default=18.0)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Checker()
    deadline = time.monotonic() + max(1.0, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        errors = []
        if node.latest_map is None:
            errors.append("no /perceptive_terrain_map received")
        else:
            resolution = float(node.latest_map.info.resolution)
            if not math.isclose(resolution, 0.04, abs_tol=1e-6):
                errors.append(f"resolution is {resolution:.4f} m, expected 0.0400 m")
            missing = sorted(REQUIRED_LAYERS.difference(node.latest_map.layers))
            if missing:
                errors.append("missing layers: " + ", ".join(missing))
        if len(node.map_times) < 2:
            errors.append("not enough processed maps to measure rate")
            rate = 0.0
        else:
            rate = (len(node.map_times) - 1) / (
                node.map_times[-1] - node.map_times[0])
            if rate < args.minimum_rate:
                errors.append(
                    f"processed-map rate is {rate:.2f} Hz, "
                    f"minimum is {args.minimum_rate:.2f} Hz")
        if node.latest_terrain is None:
            errors.append("no /planar_terrain received")
            regions = 0
        else:
            regions = len(node.latest_terrain.regions)
            if regions == 0:
                errors.append("planar terrain contains no regions")
            for region in node.latest_terrain.regions:
                if len(region.boundary.points) < 3:
                    errors.append(f"region {region.id} has an invalid boundary")
                if region.normal.z < 0.87:
                    errors.append(
                        f"region {region.id} normal.z={region.normal.z:.3f} < 0.87")

        if errors:
            for error in errors:
                node.get_logger().error(error)
            return 1
        node.get_logger().info(
            f"PASS: 4 cm map at {rate:.2f} Hz, "
            f"{regions} current planar region(s), all paper layers present")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
