import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("legged_controllers"), "config", "joy_ros2.yaml")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{
            "dev": LaunchConfiguration("joy_dev"),
            "deadzone": 1e-3,
            "autorepeat_rate": 10.0,
            "coalesce_interval": 0.05,
        }],
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[LaunchConfiguration("teleop_config")],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    return LaunchDescription([
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("teleop_config", default_value=default_config),
        joy_node,
        teleop_node,
    ])
