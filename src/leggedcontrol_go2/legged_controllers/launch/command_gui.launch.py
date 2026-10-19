from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # The real-hardware ros2_control process must use Fast DDS because
            # it preloads Unitree SDK2's private CycloneDDS ABI.  Keep the GUI
            # on the same ROS RMW so Trigger/SetBool service request-reply
            # headers are not mixed across DDS implementations.
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            DeclareLaunchArgument("simulation_backend", default_value="auto"),
            Node(
                package="legged_controllers",
                executable="legged_command_gui.py",
                name="legged_command_gui",
                output="screen",
                parameters=[{
                    "simulation_backend": LaunchConfiguration("simulation_backend"),
                }],
            )
        ]
    )
