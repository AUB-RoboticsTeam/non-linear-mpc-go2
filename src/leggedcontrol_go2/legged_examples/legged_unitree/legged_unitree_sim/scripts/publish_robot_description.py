#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node("robot_description_publisher")
    node.declare_parameter("xacro_file", "")
    node.declare_parameter("robot_type", "go2")
    node.declare_parameter("output_file", "")
    node.declare_parameter("power_limit", "")
    node.declare_parameter("contact_threshold", "")
    node.declare_parameter("domain_id", "")
    node.declare_parameter("network_interface", "")
    node.declare_parameter("state_timeout", "")
    node.declare_parameter("startup_timeout", "")
    node.declare_parameter("arm_ramp_duration", "")
    node.declare_parameter("max_torque", "")
    node.declare_parameter("max_kp", "")
    node.declare_parameter("max_kd", "")
    node.declare_parameter("stand_duration", "")
    node.declare_parameter("stand_kp", "")
    node.declare_parameter("stand_hold_kp", "")
    node.declare_parameter("stand_kd", "")
    node.declare_parameter("stand_crouch_hfe_target", "")
    node.declare_parameter("stand_crouch_kfe_target", "")
    node.declare_parameter("stand_crouch_fraction", "")
    node.declare_parameter("stand_hfe_target", "")
    node.declare_parameter("stand_kfe_target", "")
    node.declare_parameter("stand_position_tolerance", "")
    node.declare_parameter("stand_velocity_tolerance", "")
    node.declare_parameter("stand_settle_duration", "")
    node.declare_parameter("damping_kd", "")
    node.declare_parameter("release_mode_on_arm", "")
    node.declare_parameter("require_motion_mode_check", "")

    xacro_file = node.get_parameter("xacro_file").value
    robot_type = node.get_parameter("robot_type").value
    output_file = node.get_parameter("output_file").value
    optional_xacro_args = {
        "power_limit": node.get_parameter("power_limit").value,
        "contact_threshold": node.get_parameter("contact_threshold").value,
        "domain_id": node.get_parameter("domain_id").value,
        "network_interface": node.get_parameter("network_interface").value,
        "state_timeout": node.get_parameter("state_timeout").value,
        "startup_timeout": node.get_parameter("startup_timeout").value,
        "arm_ramp_duration": node.get_parameter("arm_ramp_duration").value,
        "max_torque": node.get_parameter("max_torque").value,
        "max_kp": node.get_parameter("max_kp").value,
        "max_kd": node.get_parameter("max_kd").value,
        "stand_duration": node.get_parameter("stand_duration").value,
        "stand_kp": node.get_parameter("stand_kp").value,
        "stand_hold_kp": node.get_parameter("stand_hold_kp").value,
        "stand_kd": node.get_parameter("stand_kd").value,
        "stand_crouch_hfe_target": node.get_parameter("stand_crouch_hfe_target").value,
        "stand_crouch_kfe_target": node.get_parameter("stand_crouch_kfe_target").value,
        "stand_crouch_fraction": node.get_parameter("stand_crouch_fraction").value,
        "stand_hfe_target": node.get_parameter("stand_hfe_target").value,
        "stand_kfe_target": node.get_parameter("stand_kfe_target").value,
        "stand_position_tolerance": node.get_parameter("stand_position_tolerance").value,
        "stand_velocity_tolerance": node.get_parameter("stand_velocity_tolerance").value,
        "stand_settle_duration": node.get_parameter("stand_settle_duration").value,
        "damping_kd": node.get_parameter("damping_kd").value,
        "release_mode_on_arm": node.get_parameter("release_mode_on_arm").value,
        "require_motion_mode_check": node.get_parameter("require_motion_mode_check").value,
    }
    if not xacro_file:
        node.get_logger().error("xacro_file parameter is required")
        rclpy.shutdown()
        return 1

    try:
        command = ["xacro", xacro_file, f"robot_type:={robot_type}"]
        command.extend([f"{name}:={value}" for name, value in optional_xacro_args.items() if value != ""])
        description = subprocess.check_output(command, text=True)
    except subprocess.CalledProcessError as exc:
        node.get_logger().error(f"failed to expand xacro: {exc}")
        rclpy.shutdown()
        return 1

    if output_file:
        path = Path(output_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(description)
        node.get_logger().info(f"wrote robot description to {output_file}")

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(String, "robot_description", qos)
    msg = String()
    msg.data = description
    # Discovery can take several seconds on the real robot network (especially
    # through a router).  Do not fire one sample and exit before
    # controller_manager has created its subscription.  Wait for a matched
    # reader, then publish a short burst.  TRANSIENT_LOCAL still serves late
    # readers while this process remains alive.
    discovery_deadline = time.monotonic() + 15.0
    while rclpy.ok() and publisher.get_subscription_count() == 0 and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    matched = publisher.get_subscription_count()
    if matched == 0:
        node.get_logger().warning(
            "no robot_description subscriber discovered after 15 s; publishing anyway")
    else:
        node.get_logger().info(f"robot_description has {matched} matching subscriber(s)")

    for _ in range(10):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info("published robot_description")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
