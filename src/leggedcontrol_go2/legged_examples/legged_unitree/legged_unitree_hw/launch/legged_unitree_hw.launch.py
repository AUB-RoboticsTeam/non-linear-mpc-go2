import os
import re
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def _load_hw_config(robot_type: str) -> dict:
    config_path = os.path.join(get_package_share_directory("legged_unitree_hw"), "config", f"{robot_type}.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    return config.get("legged_unitree_hw", {})


def _interface_has_ipv4(interface: str) -> bool:
    if not interface or not os.path.exists(f"/sys/class/net/{interface}"):
        return False
    result = subprocess.run(
        ["ip", "-o", "-4", "addr", "show", "dev", interface],
        check=False, capture_output=True, text=True)
    return result.returncode == 0 and bool(result.stdout.strip())


def _interface_is_wireless(interface: str) -> bool:
    return os.path.isdir(f"/sys/class/net/{interface}/wireless")


def _gnome_network_settings_is_open() -> bool:
    result = subprocess.run(
        ["pgrep", "-f", r"^/usr/bin/gnome-control-center(?:\s|$)"],
        check=False, capture_output=True, text=True)
    return result.returncode == 0


def _resolve_network_interface(requested: str, robot_ip: str = "192.168.123.161") -> str:
    if _interface_has_ipv4(requested):
        return requested

    route = subprocess.run(
        ["ip", "-4", "route", "get", robot_ip],
        check=False, capture_output=True, text=True)
    match = re.search(r"\bdev\s+(\S+)", route.stdout)
    routed = match.group(1) if match else ""
    if _interface_has_ipv4(routed):
        print(
            f"[legged_unitree_hw] requested interface '{requested}' is unavailable; "
            f"using '{routed}' from the route to {robot_ip}",
            flush=True,
        )
        return routed

    raise RuntimeError(
        f"Network interface '{requested}' has no IPv4 address and no usable route "
        f"to the Go2 at {robot_ip} was found. Connect to the robot/router first."
    )


def _launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    cheater = LaunchConfiguration("cheater").perform(context)
    start_controller = LaunchConfiguration("start_controller").perform(context)
    load_mpc_controller = LaunchConfiguration("load_mpc_controller").perform(context).lower() in ("1", "true", "yes", "on")
    allow_wireless_mpc = LaunchConfiguration("allow_wireless_mpc").perform(context).lower() in ("1", "true", "yes", "on")
    network_interface = LaunchConfiguration("network_interface").perform(context)
    domain_id = LaunchConfiguration("domain_id").perform(context)
    release_mode_on_arm = LaunchConfiguration("release_mode_on_arm").perform(context)

    hw_config = _load_hw_config(robot_type)
    power_limit = str(hw_config.get("power_limit", 4))
    contact_threshold = str(hw_config.get("contact_threshold", 30))
    if not domain_id:
        domain_id = str(hw_config.get("domain_id", 0))
    if not network_interface:
        network_interface = str(hw_config.get("network_interface", ""))
    network_interface = _resolve_network_interface(network_interface)
    wireless_mpc = load_mpc_controller and _interface_is_wireless(network_interface)
    if wireless_mpc and not allow_wireless_mpc:
        raise RuntimeError(
            f"Refusing real-time MPC on wireless interface '{network_interface}'. "
            "Wireless MPC requires an independently validated rt/lowstate link. "
            "Use load_mpc_controller:=false for diagnostics, or explicitly pass "
            "allow_wireless_mpc:=true after validating the link."
        )
    def safety_arg(name, default):
        value = LaunchConfiguration(name).perform(context)
        return value if value else str(hw_config.get(name, default))

    safety_values = {
        "state_timeout": safety_arg("state_timeout", 0.10),
        "controller_command_timeout": safety_arg("controller_command_timeout", 0.05),
        "command_stream_rate": safety_arg("command_stream_rate", 500.0),
        "startup_timeout": safety_arg("startup_timeout", 5.0),
        "arm_ramp_duration": safety_arg("arm_ramp_duration", 2.0),
        "max_torque": safety_arg("max_torque", 30.0),
        "max_kp": safety_arg("max_kp", 80.0),
        "max_kd": safety_arg("max_kd", 10.0),
        "stand_duration": safety_arg("stand_duration", 8.0),
        "stand_kp": safety_arg("stand_kp", 50.0),
        "stand_hold_kp": safety_arg("stand_hold_kp", 70.0),
        "stand_kd": safety_arg("stand_kd", 5.0),
        "stand_crouch_hfe_target": safety_arg("stand_crouch_hfe_target", 0.90),
        "stand_crouch_kfe_target": safety_arg("stand_crouch_kfe_target", -1.80),
        "stand_crouch_fraction": safety_arg("stand_crouch_fraction", 0.45),
        "stand_hfe_target": safety_arg("stand_hfe_target", 0.60),
        "stand_kfe_target": safety_arg("stand_kfe_target", -1.20),
        "stand_position_tolerance": safety_arg("stand_position_tolerance", 0.18),
        "stand_velocity_tolerance": safety_arg("stand_velocity_tolerance", 0.35),
        "stand_settle_duration": safety_arg("stand_settle_duration", 2.0),
        "damping_kd": safety_arg("damping_kd", 3.0),
        "release_mode_on_arm": release_mode_on_arm,
        "require_motion_mode_check": str(hw_config.get("require_motion_mode_check", True)).lower(),
    }

    if wireless_mpc:
        try:
            state_timeout = float(safety_values["state_timeout"])
        except ValueError as exc:
            raise RuntimeError(
                f"Invalid state_timeout '{safety_values['state_timeout']}'"
            ) from exc
        if state_timeout > 0.10:
            raise RuntimeError(
                "Wireless MPC may not use state_timeout above 0.10 s; increasing "
                "the watchdog would hide an unsafe feedback interruption."
            )
        if _gnome_network_settings_is_open():
            raise RuntimeError(
                "Close GNOME Settings before wireless MPC. Its Wi-Fi panel was "
                "observed forcing full-band scans every 15 seconds and interrupting "
                "rt/lowstate for more than 100 ms."
            )
        print(
            f"[legged_unitree_hw] WIRELESS MPC EXPLICITLY ENABLED on "
            f"'{network_interface}'; state watchdog={state_timeout:.3f} s. "
            "Keep Wi-Fi settings and network scanners closed.",
            flush=True,
        )

    description_pkg = get_package_share_directory("legged_unitree_description")
    controllers_pkg = get_package_share_directory("legged_controllers")

    xacro_file = os.path.join(description_pkg, "urdf", "robot_hw.xacro")
    urdf_file = f"/tmp/legged_control/{robot_type}.urdf"
    controller_yaml = os.path.join(controllers_pkg, "config", "controllers.yaml")

    # unitree_sdk2's ddscxx in /usr/local was built against Unitree's bundled
    # CycloneDDS C library.  ROS Jazzy also ships a different libddsc.so.0.
    # Letting the dynamic loader combine those two ABIs crashes in DDS receive
    # threads (typically `free(): invalid pointer`).  Keep ROS communication in
    # this process on Fast DDS and preload the matching Unitree C/C++ pair.
    unitree_dds_libraries = ":".join([
        "/usr/local/lib/libddsc.so.0",
        "/usr/local/lib/libddscxx.so.0",
    ])
    missing_unitree_dds = [path for path in unitree_dds_libraries.split(":") if not os.path.exists(path)]
    if missing_unitree_dds:
        raise RuntimeError(f"Missing Unitree DDS libraries: {', '.join(missing_unitree_dds)}")

    robot_description_publisher = Node(
        package="legged_unitree_sim",
        executable="publish_robot_description.py",
        output="screen",
        parameters=[
            {"xacro_file": xacro_file},
            {"robot_type": robot_type},
            {"output_file": urdf_file},
            {"power_limit": power_limit},
            {"contact_threshold": contact_threshold},
            {"domain_id": domain_id},
            {"network_interface": network_interface},
            *[{name: value} for name, value in safety_values.items()],
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                "xacro ", xacro_file,
                " robot_type:=", robot_type,
                " power_limit:=", power_limit,
                " contact_threshold:=", contact_threshold,
                " domain_id:=", domain_id,
                " network_interface:=", network_interface,
                *sum(([f" {name}:=", value] for name, value in safety_values.items()), []),
            ]),
        }],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controller_yaml],
        additional_env={
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "LD_PRELOAD": unitree_dds_libraries,
        },
    )

    load_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controllers_pkg, "launch", "load_controller.launch.py")),
        launch_arguments={
            "robot_type": robot_type,
            "cheater": cheater,
            "simulation_mode": "false",
            "enable_visualization": "false",
            # The cmd_vel target converter requires the MPC observation. Keep
            # it available at a modest rate on hardware to avoid serializing
            # an observation on every 500 Hz controller update.
            "publish_mpc_observation": "true",
            "mpc_observation_publish_rate": "20.0",
            "start_controller": start_controller,
            "urdf_file": urdf_file,
        }.items(),
    )

    state_only_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"],
    )

    post_description_actions = []
    if load_mpc_controller:
        post_description_actions.append(load_controller)
    else:
        # A real-hardware state-only test must not construct the MPC/OCS2
        # controller.  It keeps every command interface unclaimed and also
        # lets hardware bring-up and teardown be tested independently.
        post_description_actions.append(state_only_broadcaster)

    return [
        robot_description_publisher,
        robot_state_publisher,
        # Start the subscriber before the finite-lifetime description
        # publisher exits.  This removes a discovery race seen when the Go2 is
        # reached through a router instead of a direct cable.
        ros2_control_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot_description_publisher,
                on_exit=post_description_actions,
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Unitree publishes many non-ROS CycloneDDS topics on domain 0 without
        # ROS 2 type hashes.  Keeping the ROS graph on Fast DDS prevents those
        # foreign topics from flooding every helper node with parse warnings;
        # UnitreeHW continues to use SDK2's own CycloneDDS transport internally.
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        DeclareLaunchArgument("robot_type", default_value="go2"),
        DeclareLaunchArgument("cheater", default_value="false"),
        DeclareLaunchArgument("start_controller", default_value="false"),
        DeclareLaunchArgument("load_mpc_controller", default_value="true"),
        DeclareLaunchArgument("domain_id", default_value=""),
        DeclareLaunchArgument("network_interface", default_value=""),
        DeclareLaunchArgument("allow_wireless_mpc", default_value="false"),
        DeclareLaunchArgument("release_mode_on_arm", default_value="false"),
        DeclareLaunchArgument("state_timeout", default_value=""),
        DeclareLaunchArgument("controller_command_timeout", default_value=""),
        DeclareLaunchArgument("command_stream_rate", default_value=""),
        DeclareLaunchArgument("startup_timeout", default_value=""),
        DeclareLaunchArgument("arm_ramp_duration", default_value=""),
        DeclareLaunchArgument("max_torque", default_value=""),
        DeclareLaunchArgument("max_kp", default_value=""),
        DeclareLaunchArgument("max_kd", default_value=""),
        DeclareLaunchArgument("stand_duration", default_value=""),
        DeclareLaunchArgument("stand_kp", default_value=""),
        DeclareLaunchArgument("stand_hold_kp", default_value=""),
        DeclareLaunchArgument("stand_kd", default_value=""),
        DeclareLaunchArgument("stand_crouch_hfe_target", default_value=""),
        DeclareLaunchArgument("stand_crouch_kfe_target", default_value=""),
        DeclareLaunchArgument("stand_crouch_fraction", default_value=""),
        DeclareLaunchArgument("stand_hfe_target", default_value=""),
        DeclareLaunchArgument("stand_kfe_target", default_value=""),
        DeclareLaunchArgument("stand_position_tolerance", default_value=""),
        DeclareLaunchArgument("stand_velocity_tolerance", default_value=""),
        DeclareLaunchArgument("stand_settle_duration", default_value=""),
        DeclareLaunchArgument("damping_kd", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
