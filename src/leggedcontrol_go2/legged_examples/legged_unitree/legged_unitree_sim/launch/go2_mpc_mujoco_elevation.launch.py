import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("legged_unitree_sim")
    elevation_model = PathJoinSubstitution(
        [
            FindPackageShare("legged_unitree_description"),
            "mujoco",
            "go2",
            "elevation_scene.xml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("initial_base_x", default_value="-1.0"),
            DeclareLaunchArgument("initial_base_y", default_value="-1.0"),
            DeclareLaunchArgument("initial_base_z", default_value="0.45"),
            DeclareLaunchArgument("start_elevation_mapping", default_value="true"),
            DeclareLaunchArgument("start_terrain_processing", default_value="true"),
            DeclareLaunchArgument(
                "elevation_mapping_script",
                default_value="/home/user/Downloads/go2222/src/elevation_mapping_cupy_ros2_ws/launch_elevation_mapping.sh",
            ),
            DeclareLaunchArgument("start_perceptive_planner", default_value="true"),
            DeclareLaunchArgument("terrain_forward", default_value="0.14"),
            ExecuteProcess(
                cmd=[LaunchConfiguration("elevation_mapping_script")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_elevation_mapping")),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="legged_controllers",
                        executable="perceptive_terrain_processor.py",
                        name="perceptive_terrain_processor",
                        output="screen",
                        parameters=[
                            os.path.join(
                                get_package_share_directory("legged_controllers"),
                                "config",
                                "perceptive_terrain.yaml",
                            )
                        ],
                        condition=IfCondition(LaunchConfiguration("start_terrain_processing")),
                    )
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_pkg, "launch", "go2_mpc_mujoco.launch.py")
                ),
                launch_arguments={
                    "mujoco_model_path": elevation_model,
                    "initial_base_x": LaunchConfiguration("initial_base_x"),
                    "initial_base_y": LaunchConfiguration("initial_base_y"),
                    "initial_base_z": LaunchConfiguration("initial_base_z"),
                }.items(),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="legged_controllers",
                        executable="terrain_foothold_planner.py",
                        name="perceptive_foothold_planner",
                        output="screen",
                        arguments=[
                            "--continuous",
                            "--forward", LaunchConfiguration("terrain_forward"),
                            "--max-step-length", "0.18",
                            "--search-radius", "0.12",
                            "--max-traversability-cost", "0.05",
                            "--max-edge-traversability-cost", "0.10",
                        ],
                        condition=IfCondition(LaunchConfiguration("start_perceptive_planner")),
                    )
                ],
            ),
        ]
    )
