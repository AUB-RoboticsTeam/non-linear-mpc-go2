import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = get_package_share_directory("legged_unitree_sim")
    default_mujoco_model = PathJoinSubstitution(
        [FindPackageShare("legged_unitree_description"), "mujoco", "go2", "scene.xml"]
    )

    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "go2_mujoco.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_type": LaunchConfiguration("robot_type"),
            "robot_name": LaunchConfiguration("robot_name"),
            "rviz": LaunchConfiguration("rviz"),
            "mujoco_model_path": LaunchConfiguration("mujoco_model_path"),
            "initial_base_x": LaunchConfiguration("initial_base_x"),
            "initial_base_y": LaunchConfiguration("initial_base_y"),
            "initial_base_z": LaunchConfiguration("initial_base_z"),
        }.items(),
    )

    mpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "go2_mpc_controller.launch.py")),
        launch_arguments={
            "robot_type": LaunchConfiguration("robot_type"),
            "cheater": LaunchConfiguration("cheater"),
            "start_controller": LaunchConfiguration("start_controller"),
            "start_gait_command": LaunchConfiguration("start_gait_command"),
            "start_command_console": LaunchConfiguration("start_command_console"),
            "start_target_publisher": LaunchConfiguration("start_target_publisher"),
            "contact_sensor_names": "FL_foot_fts,FR_foot_fts,RL_foot_fts,RR_foot_fts",
            "publish_estimated_base_tf": "false",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot_type", default_value="go2"),
        DeclareLaunchArgument("robot_name", default_value="go2"),
        DeclareLaunchArgument("mujoco_model_path", default_value=default_mujoco_model),
        DeclareLaunchArgument("initial_base_x", default_value="0.0"),
        DeclareLaunchArgument("initial_base_y", default_value="0.0"),
        DeclareLaunchArgument("initial_base_z", default_value="0.30"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("cheater", default_value="false"),
        DeclareLaunchArgument("start_controller", default_value="false"),
        DeclareLaunchArgument("start_gait_command", default_value="true"),
        DeclareLaunchArgument("start_command_console", default_value="true"),
        DeclareLaunchArgument("start_target_publisher", default_value="true"),
        DeclareLaunchArgument("mpc_start_delay", default_value="6.0"),
        mujoco_launch,
        TimerAction(period=LaunchConfiguration("mpc_start_delay"), actions=[mpc_launch]),
    ])
