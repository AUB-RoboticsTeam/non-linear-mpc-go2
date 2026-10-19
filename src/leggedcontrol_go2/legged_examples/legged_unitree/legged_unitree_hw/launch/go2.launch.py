import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    forwarded = [
        "cheater", "start_controller", "load_mpc_controller", "domain_id", "network_interface",
        "allow_wireless_mpc", "release_mode_on_arm", "state_timeout", "startup_timeout",
        "controller_command_timeout", "command_stream_rate",
        "arm_ramp_duration", "max_torque", "max_kp", "max_kd",
        "stand_duration", "stand_kp", "stand_hold_kp", "stand_kd",
        "stand_crouch_hfe_target", "stand_crouch_kfe_target",
        "stand_crouch_fraction",
        "stand_hfe_target", "stand_kfe_target",
        "stand_position_tolerance", "stand_velocity_tolerance",
        "stand_settle_duration", "damping_kd",
    ]
    return LaunchDescription([
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("legged_unitree_hw"), "launch", "legged_unitree_hw.launch.py")
            ),
            launch_arguments={
                "robot_type": "go2",
                **{name: LaunchConfiguration(name) for name in forwarded},
            }.items(),
        )
    ])
