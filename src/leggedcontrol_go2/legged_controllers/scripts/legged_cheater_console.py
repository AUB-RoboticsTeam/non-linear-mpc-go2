#!/usr/bin/env python3

import sys
import time
import subprocess

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CheaterConsole(Node):
    def __init__(self):
        super().__init__("legged_cheater_console")
        self.declare_parameter("controller_name", "legged_cheater_controller")
        self.declare_parameter("command_topic", "/legged_robot_gait_command")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("activation_height", 0.34)

        self.controller_name = self.get_parameter("controller_name").get_parameter_value().string_value
        self.activation_height = self.get_parameter("activation_height").get_parameter_value().double_value
        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        controller_manager = self.get_parameter("controller_manager").get_parameter_value().string_value

        self.gait_pub = self.create_publisher(String, command_topic, 10)
        self.twist_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.switch_client = self.create_client(SwitchController, controller_manager + "/switch_controller")

        self.get_logger().info(
            f"console ready: gait -> {command_topic}, twist -> {cmd_vel_topic}, controller -> {self.controller_name}"
        )

    def set_initial_pose_hold(self, enabled: bool):
        value = "true" if enabled else "false"
        try:
            subprocess.run(
                [
                    "gz",
                    "topic",
                    "-t",
                    "/legged_initial_pose_hold",
                    "-m",
                    "gz.msgs.Boolean",
                    "-p",
                    f"data: {value}",
                ],
                check=True,
                timeout=2.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            state = "enabled" if enabled else "disabled"
            self.get_logger().info(f"{state} Gazebo initial-pose hold")
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            self.get_logger().warn(f"could not publish Gazebo initial-pose hold command: {exc}")

    def set_gazebo_paused(self, paused: bool):
        value = "true" if paused else "false"
        try:
            subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    "/world/default/control",
                    "--reqtype",
                    "gz.msgs.WorldControl",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "1000",
                    "--req",
                    f"pause: {value}",
                ],
                check=True,
                timeout=2.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            state = "paused" if paused else "running"
            self.get_logger().info(f"Gazebo is {state}")
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            self.get_logger().warn(f"could not set Gazebo pause state: {exc}")

    def reset_model_pose(self):
        try:
            subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    "/world/default/set_pose",
                    "--reqtype",
                    "gz.msgs.Pose",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "1000",
                    "--req",
                    (
                        f'name: "go2", position {{ x: 0 y: 0 z: {self.activation_height:.6f} }}, '
                        "orientation { w: 1 }"
                    ),
                ],
                check=True,
                timeout=2.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info(f"reset Gazebo model pose to z={self.activation_height:.3f}")
        except (subprocess.SubprocessError, FileNotFoundError) as exc:
            self.get_logger().warn(f"could not reset Gazebo model pose: {exc}")

    def publish_gait(self, gait: str):
        msg = String()
        msg.data = gait
        self.gait_pub.publish(msg)
        self.get_logger().info(f"sent gait command '{gait}'")

    def publish_twist(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self.twist_pub.publish(msg)
        self.get_logger().info(f"sent cmd_vel vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")

    def publish_zero_burst(self, duration_sec: float = 1.0):
        end_time = time.monotonic() + duration_sec
        msg = Twist()
        while time.monotonic() < end_time and rclpy.ok():
            self.twist_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)
        self.get_logger().info("sent zero cmd_vel burst")

    def switch_controller(self, activate: bool):
        if activate:
            self.set_gazebo_paused(True)
            self.reset_model_pose()
            self.set_initial_pose_hold(True)
            self.publish_gait("stance")
            self.publish_zero_burst(0.3)
        if not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/controller_manager/switch_controller is not available")
            return

        request = SwitchController.Request()
        request.activate_controllers = [self.controller_name] if activate else []
        request.deactivate_controllers = [] if activate else [self.controller_name]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout = Duration(sec=5, nanosec=0)

        future = self.switch_client.call_async(request)
        if activate:
            # The Gazebo controller manager completes switches from its update loop,
            # so physics must be running while the request is pending. Keep the
            # initial joint hold enabled until the controller reports active.
            self.set_gazebo_paused(False)
        rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("controller switch timed out")
            if activate:
                self.set_gazebo_paused(True)
            return
        if future.result().ok:
            action = "activated" if activate else "deactivated"
            self.get_logger().info(f"{action} {self.controller_name}")
            if activate:
                self.publish_gait("stance")
                self.set_initial_pose_hold(False)
                self.publish_zero_burst(1.0)
            if not activate:
                self.set_gazebo_paused(True)
                self.set_initial_pose_hold(True)
        else:
            self.get_logger().error(f"controller switch failed: {future.result().message}")


HELP = """Commands:
  activate | start          activate the selected legged controller
  deactivate                deactivate the selected legged controller
  pause | unpause           pause or unpause Gazebo physics
  hold on | hold off        enable or disable Gazebo initial-pose hold
  list                      list available gaits
  <gait_name>               send gait, for example: trot
  twist <vx> <vy> <wz>      publish /cmd_vel, for example: twist 0.2 0 0
  zero                      publish zero /cmd_vel
  stand                     send stance and a short zero /cmd_vel burst
  help                      show this help
  quit                      exit this console
"""


def _handle_command(node: CheaterConsole, line: str) -> bool:
    words = line.strip().split()
    if not words:
        return True
    command = words[0].lower()

    if command in ("quit", "exit"):
        return False
    if command == "help":
        print(HELP)
    elif command in ("activate", "start"):
        node.switch_controller(True)
    elif command == "deactivate":
        node.switch_controller(False)
    elif command == "pause":
        node.set_gazebo_paused(True)
    elif command in ("unpause", "run"):
        node.set_gazebo_paused(False)
    elif command == "hold":
        if len(words) != 2 or words[1].lower() not in ("on", "off"):
            print("usage: hold on|off")
        else:
            node.set_initial_pose_hold(words[1].lower() == "on")
    elif command in ("twist", "cmd_vel", "vel"):
        if len(words) != 4:
            print("usage: twist <vx> <vy> <wz>")
        else:
            node.publish_twist(float(words[1]), float(words[2]), float(words[3]))
    elif command in ("zero", "stop"):
        node.publish_twist(0.0, 0.0, 0.0)
    elif command == "stand":
        node.publish_gait("stance")
        node.publish_zero_burst(1.0)
    else:
        node.publish_gait(command)

    rclpy.spin_once(node, timeout_sec=0.05)
    return True


def main(args=None):
    rclpy.init(args=args)
    node = CheaterConsole()
    time.sleep(0.2)
    try:
        print(HELP)
        while rclpy.ok():
            try:
                line = input("cheater> ")
            except EOFError:
                break
            try:
                if not _handle_command(node, line):
                    break
            except ValueError as exc:
                print(f"invalid number: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
