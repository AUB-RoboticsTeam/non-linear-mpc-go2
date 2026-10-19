import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_setup(context, *args, **kwargs):
    sim_backend = LaunchConfiguration("sim_backend").perform(context)
    sim_pkg = get_package_share_directory("legged_unitree_sim")

    if sim_backend == "mujoco":
        sim_launch = os.path.join(sim_pkg, "launch", "go2_mujoco.launch.py")
        launch_arguments = {
            "robot_type": LaunchConfiguration("robot_type"),
            "rviz": LaunchConfiguration("rviz"),
        }
    elif sim_backend == "gazebo":
        sim_launch = os.path.join(sim_pkg, "launch", "go2_gz.launch.py")
        launch_arguments = {
            "robot_type": LaunchConfiguration("robot_type"),
            "rviz": LaunchConfiguration("rviz"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
        }
    else:
        raise ValueError(f"Unknown sim_backend '{sim_backend}', expected 'gazebo' or 'mujoco'")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments=launch_arguments.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="go2"),
        DeclareLaunchArgument(
            "sim_backend", default_value="gazebo",
            description="Simulator to spawn the robot in: gazebo or mujoco",
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.42"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
