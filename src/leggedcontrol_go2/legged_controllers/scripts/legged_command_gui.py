#!/usr/bin/env python3
"""Desktop command panel for the ROS 2 legged controller."""

import math
import subprocess
import sys
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


GAITS = (
    "stance", "trot", "standing_trot", "flying_trot", "pace",
    "standing_pace", "dynamic_walk", "static_walk", "amble",
    "lindyhop", "skipping", "pawup", "stair_crawl",
)


class CommandNode(Node):
    def __init__(self):
        super().__init__("legged_command_gui")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("gait_topic", "/legged_robot_gait_command")
        self.declare_parameter("simulation_backend", "auto")
        self.declare_parameter("gazebo_world_name", "default")
        self.declare_parameter("gazebo_robot_name", "go2")
        self.declare_parameter("gazebo_activation_x", 0.0)
        self.declare_parameter("gazebo_activation_y", 0.0)
        self.declare_parameter("gazebo_activation_z", 0.34)
        self.declare_parameter("gazebo_activation_yaw", 0.0)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        gait_topic = str(self.get_parameter("gait_topic").value)
        self.simulation_backend = str(
            self.get_parameter("simulation_backend").value).lower()
        self.gazebo_world_name = str(
            self.get_parameter("gazebo_world_name").value)
        self.gazebo_robot_name = str(
            self.get_parameter("gazebo_robot_name").value)
        self.gazebo_activation_x = float(
            self.get_parameter("gazebo_activation_x").value)
        self.gazebo_activation_y = float(
            self.get_parameter("gazebo_activation_y").value)
        self.gazebo_activation_z = float(
            self.get_parameter("gazebo_activation_z").value)
        self.gazebo_activation_yaw = float(
            self.get_parameter("gazebo_activation_yaw").value)
        self.cmd_publisher = self.create_publisher(Twist, cmd_topic, 10)
        self.gait_publisher = self.create_publisher(String, gait_topic, 10)
        self.stand_client = self.create_client(Trigger, "/unitree_hw/stand_up")
        self.damp_client = self.create_client(Trigger, "/unitree_hw/damp")
        self.enable_client = self.create_client(
            SetBool, "/unitree_hw/enable_commands"
        )
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.stand_state = "unknown"
        self.commands_enabled = False
        self.joystick = None
        self.joystick_received_at = 0.0
        hardware_status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/unitree_hw/stand_state", self._stand_state_callback,
            hardware_status_qos,
        )
        self.create_subscription(
            Bool, "/unitree_hw/commands_enabled",
            self._commands_enabled_callback, hardware_status_qos,
        )
        self.create_subscription(
            Joy, "/unitree_hw/joystick", self._joystick_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.get_logger().info(
            f"command GUI ready: velocity -> {cmd_topic}, gait -> {gait_topic}"
        )

    def publish_twist(self, x, y, z, yaw):
        message = Twist()
        message.linear.x = float(x)
        message.linear.y = float(y)
        message.linear.z = float(z)
        message.angular.z = float(yaw)
        self.cmd_publisher.publish(message)

    def publish_gait(self, gait):
        self.gait_publisher.publish(String(data=str(gait)))

    def _stand_state_callback(self, message):
        self.stand_state = message.data

    def _commands_enabled_callback(self, message):
        self.commands_enabled = bool(message.data)

    def _joystick_callback(self, message):
        self.joystick = message
        self.joystick_received_at = time.monotonic()

    def joystick_is_fresh(self):
        return (
            self.joystick is not None
            and time.monotonic() - self.joystick_received_at < 0.30
        )

    def _run_gz(self, arguments, action):
        try:
            result = subprocess.run(
                ["gz", *arguments], check=False, timeout=3.0,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (FileNotFoundError, subprocess.SubprocessError) as error:
            self.get_logger().error(f"Gazebo {action} failed: {error}")
            return False
        if result.returncode != 0:
            self.get_logger().error(f"Gazebo {action} failed")
            return False
        return True

    def set_gazebo_paused(self, paused):
        value = "true" if paused else "false"
        return self._run_gz([
            "service", "-s", f"/world/{self.gazebo_world_name}/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
            "--req", f"pause: {value}",
        ], f"pause={value}")

    def set_gazebo_hold(self, enabled):
        value = "true" if enabled else "false"
        return self._run_gz([
            "topic", "-t", "/legged_initial_pose_hold",
            "-m", "gz.msgs.Boolean", "-p", f"data: {value}",
        ], f"initial hold={value}")

    def reset_gazebo_pose(self):
        half_yaw = 0.5 * self.gazebo_activation_yaw
        request = (
            f'name: "{self.gazebo_robot_name}", position {{ '
            f'x: {self.gazebo_activation_x:.6f} '
            f'y: {self.gazebo_activation_y:.6f} '
            f'z: {self.gazebo_activation_z:.6f} }}, orientation {{ '
            f'z: {math.sin(half_yaw):.9f} w: {math.cos(half_yaw):.9f} }}')
        return self._run_gz([
            "service", "-s", f"/world/{self.gazebo_world_name}/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000", "--req", request,
        ], "pose reset")

    def prepare_gazebo_stand(self):
        return (
            self.set_gazebo_paused(True)
            and self.reset_gazebo_pose()
            and self.set_gazebo_hold(True)
        )

    def using_gazebo(self):
        if self.simulation_backend == "gazebo":
            return True
        if self.simulation_backend != "auto":
            return False
        try:
            result = subprocess.run(
                ["gz", "service", "-l"], check=False, timeout=2.0,
                capture_output=True, text=True)
            return (
                result.returncode == 0
                and f"/world/{self.gazebo_world_name}/control" in result.stdout
            )
        except (FileNotFoundError, subprocess.SubprocessError):
            return False


class CommandPanel:
    PUBLISH_PERIOD_MS = 100

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Go2 MPC Command Panel")
        self.root.geometry("720x690")
        self.root.minsize(640, 620)
        self.root.configure(bg="#111820")

        self.armed = False
        self.closing = False
        self.motion = (0.0, 0.0, 0.0)
        self.keys_down = set()
        self.joystick_override = False
        self.previous_joystick_buttons = [0] * 16
        self.speed_labels = {}

        # Conservative defaults are intentional: the same GUI is used for the
        # first supported real-hardware locomotion test.
        self.speed = tk.DoubleVar(value=0.05)
        self.lateral_speed = tk.DoubleVar(value=0.03)
        self.yaw_speed = tk.DoubleVar(value=0.15)
        self.height = tk.DoubleVar(value=0.34)
        self.gait = tk.StringVar(value="trot")
        self.status = tk.StringVar(
            value="DISARMED - press STAND before driving"
        )
        self.link_status = tk.StringVar(value="Waiting for ROS subscribers")
        self.command_readout = tk.StringVar(
            value="vx +0.00   vy +0.00   wz +0.00   z 0.34"
        )

        self._configure_style()
        self._build()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(self.PUBLISH_PERIOD_MS, self._publish_cycle)
        self.root.after(500, self._update_link_status)
        self.root.after(20, self._spin_ros)
        self.stand_workflow = "idle"
        self.stand_request_future = None
        self.arm_attempts = 0

    def _configure_style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("Panel.TFrame", background="#18232d")
        style.configure(
            "Title.TLabel", background="#111820", foreground="#f3f6f8",
            font=("DejaVu Sans", 20, "bold"),
        )
        style.configure(
            "Sub.TLabel", background="#111820", foreground="#8fa5b5",
            font=("DejaVu Sans", 10),
        )
        style.configure(
            "Panel.TLabel", background="#18232d", foreground="#dce6ec",
            font=("DejaVu Sans", 10),
        )
        style.configure(
            "Status.TLabel", background="#18232d", foreground="#57d7a0",
            font=("DejaVu Sans Mono", 10, "bold"),
        )
        style.configure(
            "TButton", font=("DejaVu Sans", 10, "bold"), padding=(12, 9),
            background="#2a3a46", foreground="#f4f7f8",
        )
        style.map(
            "TButton",
            background=[("active", "#3a5060"), ("pressed", "#4e6a7d")],
        )
        style.configure(
            "Drive.TButton", font=("DejaVu Sans", 12, "bold"),
            padding=(16, 15), background="#24465c",
        )
        style.configure(
            "Stop.TButton", font=("DejaVu Sans", 12, "bold"),
            padding=(16, 15), background="#b43a43",
        )
        style.map(
            "Stop.TButton",
            background=[("active", "#d34b55"), ("pressed", "#8f2931")],
        )
        style.configure(
            "Stand.TButton", font=("DejaVu Sans", 12, "bold"),
            padding=(16, 12), background="#247654",
        )
        style.map(
            "Stand.TButton",
            background=[("active", "#31976e"), ("pressed", "#195c40")],
        )
        style.configure(
            "TCombobox", fieldbackground="#22313d", foreground="#f4f7f8",
            padding=6,
        )

    def _panel(self, parent, title):
        frame = ttk.Frame(parent, style="Panel.TFrame", padding=16)
        frame.columnconfigure(0, weight=1)
        ttk.Label(
            frame, text=title, style="Panel.TLabel",
            font=("DejaVu Sans", 12, "bold"),
        ).grid(row=0, column=0, sticky="w", pady=(0, 12))
        return frame

    def _build(self):
        root = ttk.Frame(self.root, padding=20, style="Panel.TFrame")
        root.pack(fill="both", expand=True, padx=14, pady=14)
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)

        header = tk.Frame(root, bg="#111820")
        header.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 14))
        ttk.Label(header, text="GO2 / MPC", style="Title.TLabel").pack(
            anchor="w"
        )
        ttk.Label(
            header, text="ROS 2 locomotion command panel", style="Sub.TLabel"
        ).pack(anchor="w")

        safety = self._panel(root, "ROBOT STATE")
        safety.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 12))
        controls = ttk.Frame(safety, style="Panel.TFrame")
        controls.grid(row=1, column=0, sticky="ew")
        controls.columnconfigure((0, 1, 2), weight=1)
        ttk.Button(
            controls, text="STAND", style="Stand.TButton", command=self.stand
        ).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(
            controls, text="STOP MOTION", command=self.stop_motion
        ).grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Button(
            controls, text="EMERGENCY STOP", style="Stop.TButton",
            command=self.emergency_stop,
        ).grid(row=0, column=2, sticky="ew", padx=(6, 0))
        ttk.Label(
            safety, textvariable=self.status, style="Status.TLabel"
        ).grid(row=2, column=0, sticky="w", pady=(12, 0))
        ttk.Label(
            safety, textvariable=self.link_status, style="Panel.TLabel"
        ).grid(row=3, column=0, sticky="w", pady=(4, 0))

        gait_panel = self._panel(root, "GAIT")
        gait_panel.grid(
            row=2, column=0, sticky="nsew", padx=(0, 6), pady=(0, 12)
        )
        ttk.Combobox(
            gait_panel, textvariable=self.gait, values=GAITS, state="readonly"
        ).grid(row=1, column=0, sticky="ew", pady=(0, 10))
        ttk.Button(
            gait_panel, text="APPLY GAIT", command=self.apply_gait
        ).grid(row=2, column=0, sticky="ew")

        posture = self._panel(root, "POSTURE")
        posture.grid(
            row=2, column=1, sticky="nsew", padx=(6, 0), pady=(0, 12)
        )
        ttk.Label(
            posture, text="Body height", style="Panel.TLabel"
        ).grid(row=1, column=0, sticky="w")
        ttk.Scale(
            posture, variable=self.height, from_=0.20, to=0.42,
            command=self._slider_changed,
        ).grid(row=2, column=0, sticky="ew")
        self.height_label = ttk.Label(
            posture, text="0.30 m", style="Status.TLabel"
        )
        self.height_label.grid(row=3, column=0, sticky="e", pady=(4, 0))

        drive = self._panel(
            root, "DRIVE - GUI: buttons/W A S D/Q E | REMOTE: hold R1"
        )
        drive.grid(row=3, column=0, columnspan=2, sticky="nsew", pady=(0, 12))
        pad = ttk.Frame(drive, style="Panel.TFrame")
        pad.grid(row=1, column=0, sticky="n")
        for column in range(5):
            pad.columnconfigure(column, weight=1)
        self._drive_button(pad, "ROTATE LEFT\nQ", 0, 0, (0, 0, 1))
        self._drive_button(pad, "FORWARD\nW", 0, 2, (1, 0, 0))
        self._drive_button(pad, "ROTATE RIGHT\nE", 0, 4, (0, 0, -1))
        self._drive_button(pad, "LEFT\nA", 1, 1, (0, 1, 0))
        ttk.Button(
            pad, text="STOP\nSPACE", style="Stop.TButton",
            command=self.stop_motion,
        ).grid(row=1, column=2, padx=5, pady=5, sticky="nsew")
        self._drive_button(pad, "RIGHT\nD", 1, 3, (0, -1, 0))
        self._drive_button(pad, "BACKWARD\nS", 2, 2, (-1, 0, 0))
        ttk.Label(
            drive, textvariable=self.command_readout, style="Status.TLabel"
        ).grid(row=2, column=0, pady=(12, 0))

        speeds = self._panel(root, "SPEED LIMITS")
        speeds.grid(row=4, column=0, columnspan=2, sticky="ew")
        speeds.columnconfigure((0, 1, 2), weight=1)
        self._scale(speeds, 0, "Forward", self.speed, 0.02, 0.50)
        self._scale(speeds, 1, "Lateral", self.lateral_speed, 0.02, 0.35)
        self._scale(speeds, 2, "Yaw", self.yaw_speed, 0.10, 1.50)
        speed_buttons = ttk.Frame(speeds, style="Panel.TFrame")
        speed_buttons.grid(row=2, column=0, columnspan=3, pady=(14, 0))
        ttk.Button(
            speed_buttons, text="SPEED -",
            command=lambda: self._change_speed(1.0 / 1.20),
        ).grid(row=0, column=0, padx=6)
        ttk.Button(
            speed_buttons, text="SPEED +",
            command=lambda: self._change_speed(1.20),
        ).grid(row=0, column=1, padx=6)
        ttk.Label(
            speed_buttons,
            text="Remote: R1 + D-pad down/up",
            style="Panel.TLabel",
        ).grid(row=0, column=2, padx=(14, 0))

    def _scale(self, parent, column, title, variable, low, high):
        frame = ttk.Frame(parent, style="Panel.TFrame")
        frame.grid(row=1, column=column, sticky="ew", padx=8)
        frame.columnconfigure(0, weight=1)
        label = ttk.Label(
            frame, text=f"{title}: {variable.get():.2f}", style="Panel.TLabel"
        )
        label.grid(row=0, column=0, sticky="w")
        self.speed_labels[title] = label
        scale = ttk.Scale(frame, variable=variable, from_=low, to=high)
        scale.configure(
            command=lambda _value, v=variable, l=label, n=title:
            l.configure(text=f"{n}: {v.get():.2f}")
        )
        scale.grid(row=1, column=0, sticky="ew", pady=(5, 0))

    def _change_speed(self, multiplier):
        limits = (
            ("Forward", self.speed, 0.02, 0.50),
            ("Lateral", self.lateral_speed, 0.02, 0.35),
            ("Yaw", self.yaw_speed, 0.10, 1.50),
        )
        for title, variable, low, high in limits:
            value = max(low, min(high, variable.get() * multiplier))
            variable.set(value)
            self.speed_labels[title].configure(text=f"{title}: {value:.2f}")
        self.status.set(
            f"Speed limits: forward {self.speed.get():.2f}, "
            f"lateral {self.lateral_speed.get():.2f}, "
            f"yaw {self.yaw_speed.get():.2f}"
        )

    def _drive_button(self, parent, text, row, column, direction):
        button = ttk.Button(parent, text=text, style="Drive.TButton")
        button.grid(row=row, column=column, padx=5, pady=5, sticky="nsew")
        button.bind(
            "<ButtonPress-1>",
            lambda _event, selected=direction: self.set_motion(*selected),
        )
        button.bind("<ButtonRelease-1>", lambda _event: self.stop_motion())

    def _bind_keys(self):
        self.root.bind_all("<KeyPress>", self._key_press)
        self.root.bind_all("<KeyRelease>", self._key_release)
        self.root.focus_set()

    def _key_press(self, event):
        key = event.keysym.lower()
        if key == "space":
            self.stop_motion()
        elif key in {"w", "a", "s", "d", "q", "e"}:
            self.keys_down.add(key)
            self._motion_from_keys()

    def _key_release(self, event):
        key = event.keysym.lower()
        if key in self.keys_down:
            self.keys_down.discard(key)
            self._motion_from_keys()

    def _motion_from_keys(self):
        x = int("w" in self.keys_down) - int("s" in self.keys_down)
        y = int("a" in self.keys_down) - int("d" in self.keys_down)
        yaw = int("q" in self.keys_down) - int("e" in self.keys_down)
        self.set_motion(x, y, yaw)

    def _slider_changed(self, _value):
        self.height_label.configure(text=f"{self.height.get():.2f} m")
        self._update_readout()

    def _require_armed(self):
        if self.armed:
            return True
        self.status.set("DISARMED - press STAND before driving")
        self.root.bell()
        return False

    def stand(self):
        self.keys_down.clear()
        self.motion = (0.0, 0.0, 0.0)
        if self.node.stand_client.service_is_ready():
            if self.stand_workflow != "idle":
                self.status.set("Stand/MPC handoff is already in progress")
                return
            self.armed = False
            if self.node.stand_state == "ready":
                # A hardware stand-up may have been completed before the GUI
                # was opened. Continue directly with the controller handoff;
                # do not unnecessarily replay the stand trajectory.
                self.stand_workflow = "standing"
                self.status.set("Stand already ready - starting MPC handoff...")
            else:
                self.stand_workflow = "starting"
                self.status.set("REAL ROBOT - requesting low-level stand-up...")
                future = self.node.stand_client.call_async(Trigger.Request())
                self.stand_request_future = future
                future.add_done_callback(self._stand_response)
                self.root.after(15000, self._stand_request_timeout, future)
        elif self.node.using_gazebo():
            if self.stand_workflow != "idle":
                self.status.set("Gazebo stand/MPC activation is already in progress")
                return
            if self.armed:
                self.node.publish_gait("stance")
                self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
                self.status.set(
                    f"GAZEBO - stance at {self.height.get():.2f} m")
                return
            self.armed = False
            self.stand_workflow = "gazebo_activating"
            self.status.set("GAZEBO - preparing safe MPC activation...")
            if not self.node.prepare_gazebo_stand():
                self._workflow_failed("could not pause/reset/hold Gazebo")
                return
            self.node.publish_gait("stance")
            for _ in range(5):
                self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
            if not self.node.switch_client.service_is_ready():
                self._workflow_failed("controller switch service is unavailable")
                return
            request = SwitchController.Request()
            request.activate_controllers = [
                "legged_controller", "joint_state_broadcaster",
                "imu_sensor_broadcaster", "FR_foot_fts_broadcaster",
                "FL_foot_fts_broadcaster", "RR_foot_fts_broadcaster",
                "RL_foot_fts_broadcaster",
            ]
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.activate_asap = False
            request.timeout.sec = 15
            future = self.node.switch_client.call_async(request)
            # The switch completes only from gz_ros2_control's update loop.
            if not self.node.set_gazebo_paused(False):
                self._workflow_failed("could not unpause Gazebo")
                return
            future.add_done_callback(self._gazebo_controller_response)
        else:
            # Simulation starts in the nominal standing joint pose, so it only
            # needs the stance schedule and desired body height.
            self.armed = True
            self.node.publish_gait("stance")
            self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
            self.status.set(
                f"SIMULATION - standing at {self.height.get():.2f} m"
            )
        self._update_readout()

    def _gazebo_controller_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self.node.set_gazebo_paused(True)
            self._workflow_failed(f"Gazebo controller activation failed: {error}")
            return
        if not response.ok:
            self.node.set_gazebo_paused(True)
            self._workflow_failed(response.message or "controller rejected")
            return
        self.node.publish_gait("stance")
        for _ in range(5):
            self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
        self.status.set("GAZEBO MPC active - settling into stance...")
        self.root.after(1000, self._finish_gazebo_stand)

    def _finish_gazebo_stand(self):
        if self.closing or self.stand_workflow != "gazebo_activating":
            return
        if not self.node.set_gazebo_hold(False):
            self.node.set_gazebo_paused(True)
            self._workflow_failed("could not release Gazebo initial-pose hold")
            return
        self.node.publish_gait("stance")
        self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
        self.stand_workflow = "idle"
        self.armed = True
        self.status.set(
            f"GAZEBO MPC ARMED - stance at {self.height.get():.2f} m")

    def _stand_response(self, future):
        if future is not self.stand_request_future:
            return
        self.stand_request_future = None
        try:
            response = future.result()
        except Exception as error:
            self._workflow_failed(f"stand service failed: {error}")
            return
        if not response.success:
            self._workflow_failed(response.message)
            return
        self.stand_workflow = "standing"
        self.status.set("REAL ROBOT - standing up; keep clear and supported")

    def _stand_request_timeout(self, future):
        if future is not self.stand_request_future or future.done():
            return
        self.stand_request_future = None
        future.cancel()
        self._workflow_failed(
            "stand service timed out; check rt/lowstate and DDS connectivity"
        )

    def _continue_real_stand_workflow(self):
        if self.stand_workflow == "standing" and self.node.stand_state == "ready":
            self.stand_workflow = "activating"
            self.status.set("Stand ready - activating MPC controller...")
            request = SwitchController.Request()
            request.activate_controllers = ["legged_controller"]
            request.strictness = SwitchController.Request.BEST_EFFORT
            request.activate_asap = True
            request.timeout.sec = 5
            future = self.node.switch_client.call_async(request)
            future.add_done_callback(self._controller_response)

    def _controller_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._workflow_failed(f"controller activation failed: {error}")
            return
        if not response.ok:
            self._workflow_failed(response.message or "controller rejected")
            return
        self.node.publish_gait("stance")
        for _ in range(3):
            self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
        self.stand_workflow = "arming"
        self.arm_attempts = 0
        self.status.set("MPC active in stance - preparing smooth handoff...")
        self.root.after(1000, self._try_arm_mpc)

    def _try_arm_mpc(self):
        if self.closing or self.stand_workflow != "arming":
            return
        if not self.node.enable_client.service_is_ready():
            self._workflow_failed("enable_commands service disappeared")
            return
        self.arm_attempts += 1
        request = SetBool.Request()
        request.data = True
        future = self.node.enable_client.call_async(request)
        future.add_done_callback(self._arm_response)

    def _arm_response(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._workflow_failed(f"MPC handoff failed: {error}")
            return
        if not response.success:
            retryable = (
                "finite controller command" in response.message
                or "stand pose is no longer settled" in response.message
                or "stand-up is not complete" in response.message
            )
            if self.arm_attempts < 80 and retryable:
                self.status.set(
                    "Stand is reconverging - waiting before MPC handoff..."
                )
                self.root.after(250, self._try_arm_mpc)
                return
            self._workflow_failed(response.message)
            return
        self.stand_workflow = "idle"
        self.armed = True
        self.status.set(
            f"REAL ROBOT MPC ARMED - stance at {self.height.get():.2f} m"
        )

    def _workflow_failed(self, message):
        self.stand_request_future = None
        self.armed = False
        self.stand_workflow = "idle"
        self.status.set(f"STAND FAILED - {message}")
        self.root.bell()

    def apply_gait(self):
        if not self._require_armed():
            return
        selected = self.gait.get()
        self.node.publish_gait(selected)
        self.status.set(f"ARMED - gait: {selected}")

    def set_motion(self, x_sign, y_sign, yaw_sign):
        if not self._require_armed():
            return
        x = float(x_sign) * self.speed.get()
        y = float(y_sign) * self.lateral_speed.get()
        yaw = float(yaw_sign) * self.yaw_speed.get()
        if x and y:
            x /= math.sqrt(2.0)
            y /= math.sqrt(2.0)
        self.motion = (x, y, yaw)
        self.status.set(f"ARMED - gait: {self.gait.get()}")
        self._update_readout()

    def stop_motion(self):
        self.keys_down.clear()
        self.motion = (0.0, 0.0, 0.0)
        if self.armed:
            self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
            self.status.set(f"ARMED - stopped, gait: {self.gait.get()}")
        self._update_readout()

    def emergency_stop(self):
        self.keys_down.clear()
        self.motion = (0.0, 0.0, 0.0)
        for _ in range(5):
            self.node.publish_twist(0.0, 0.0, self.height.get(), 0.0)
        self.node.publish_gait("stance")
        self.armed = False
        self.stand_workflow = "idle"
        if self.node.damp_client.service_is_ready():
            self.node.damp_client.call_async(Trigger.Request())
            self.status.set("EMERGENCY STOP - continuous hardware damping")
        else:
            self.status.set("EMERGENCY STOP - command stream disarmed")
        self._update_readout()

    def _update_readout(self):
        x, y, yaw = self._active_motion()
        self.command_readout.set(
            f"vx {x:+.2f}   vy {y:+.2f}   wz {yaw:+.2f}   "
            f"z {self.height.get():.2f}"
        )

    def _publish_cycle(self):
        if self.closing:
            return
        if self.armed:
            x, y, yaw = self._active_motion()
            self.node.publish_twist(x, y, self.height.get(), yaw)
            self._update_readout()
        self.root.after(self.PUBLISH_PERIOD_MS, self._publish_cycle)

    @staticmethod
    def _deadzone(value, threshold=0.12):
        value = max(-1.0, min(1.0, float(value)))
        magnitude = abs(value)
        if magnitude <= threshold:
            return 0.0
        scaled = (magnitude - threshold) / (1.0 - threshold)
        return math.copysign(scaled, value)

    def _active_motion(self):
        joy = self.node.joystick
        fresh = self.node.joystick_is_fresh()
        r1_held = (
            fresh and len(joy.buttons) > 0 and bool(joy.buttons[0])
        )

        if r1_held:
            self.joystick_override = True
            buttons = list(joy.buttons)
            buttons.extend([0] * (16 - len(buttons)))
            # Unitree key bits 12 and 14 are D-pad up and down.  Act only on
            # the rising edge so holding a button does not race to the limit.
            if buttons[12] and not self.previous_joystick_buttons[12]:
                self._change_speed(1.20)
            if buttons[14] and not self.previous_joystick_buttons[14]:
                self._change_speed(1.0 / 1.20)
            self.previous_joystick_buttons = buttons[:16]
            if len(joy.axes) < 3:
                return (0.0, 0.0, 0.0)
            # axes: [left_x, left_y, right_x, right_y].  Signs match the
            # Unitree SDK: forward=ly, left=-lx, CCW yaw=-rx.
            x = self._deadzone(joy.axes[1]) * self.speed.get()
            y = -self._deadzone(joy.axes[0]) * self.lateral_speed.get()
            yaw = -self._deadzone(joy.axes[2]) * self.yaw_speed.get()
            self.status.set(f"ARMED - REMOTE R1 control, gait: {self.gait.get()}")
            return (x, y, yaw)

        if self.joystick_override:
            # A released R1 or a stale radio packet must produce a stop before
            # control is handed back to GUI input.
            self.joystick_override = False
            self.motion = (0.0, 0.0, 0.0)
            self.keys_down.clear()
            self.status.set(f"ARMED - remote released; stopped, gait: {self.gait.get()}")
        if fresh:
            buttons = list(joy.buttons)
            buttons.extend([0] * (16 - len(buttons)))
            self.previous_joystick_buttons = buttons[:16]
        return self.motion

    def _update_link_status(self):
        if self.closing:
            return
        velocity = self.node.cmd_publisher.get_subscription_count()
        gait = self.node.gait_publisher.get_subscription_count()
        hardware = "real HW" if self.node.stand_client.service_is_ready() else "simulation"
        remote = "live" if self.node.joystick_is_fresh() else "not detected"
        self.link_status.set(
            f"ROS links: cmd_vel={velocity}, gait={gait}; {hardware}; "
            f"stand={self.node.stand_state}; motor_mpc={self.node.commands_enabled}; "
            f"remote={remote}"
        )
        self.root.after(1000, self._update_link_status)

    def _spin_ros(self):
        if self.closing or not rclpy.ok():
            if not self.closing:
                self.closing = True
                self.root.destroy()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except ExternalShutdownException:
            self.closing = True
            self.root.destroy()
            return
        self._continue_real_stand_workflow()
        self.root.after(20, self._spin_ros)

    def close(self):
        if self.closing:
            return
        self.emergency_stop()
        self.closing = True
        self.root.after(100, self.root.destroy)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    panel = None
    try:
        panel = CommandPanel(node)
        panel.run()
    except tk.TclError as error:
        node.get_logger().error(f"cannot open GUI: {error}")
        return 1
    except KeyboardInterrupt:
        if panel is not None:
            panel.emergency_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
