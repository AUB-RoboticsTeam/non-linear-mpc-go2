import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_type = LaunchConfiguration("robot_type")

    description_pkg = FindPackageShare("legged_unitree_description")
    model_path = PathJoinSubstitution([description_pkg, "urdf", "robot_mujoco.xacro"])
    default_mujoco_model = PathJoinSubstitution([description_pkg, "mujoco", "go2", "scene.xml"])
    ros_control_config = os.path.join(
        get_package_share_directory("legged_unitree_sim"), "config", "mujoco_ros2_control.yaml"
    )
    rviz_config = os.path.join(
        get_package_share_directory("legged_unitree_sim"), "config", "go2_mujoco_lidar.rviz"
    )

    robot_description = {
        "robot_description": Command(["xacro ", model_path, " robot_type:=", robot_type])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # mujoco_ros2_control both steps physics and hosts the controller_manager
    # in-process, unlike gz_ros2_control there is no separate "spawn the
    # robot" step and no headless flag today, so the MuJoCo GLFW viewer
    # window always opens.
    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            ros_control_config,
            {"mujoco_model_path": LaunchConfiguration("mujoco_model_path")},
            {"use_sim_time": use_sim_time},
            {
                "initial_base_x": ParameterValue(
                    LaunchConfiguration("initial_base_x"), value_type=float
                ),
                "initial_base_y": ParameterValue(
                    LaunchConfiguration("initial_base_y"), value_type=float
                ),
                "initial_base_z": ParameterValue(
                    LaunchConfiguration("initial_base_z"), value_type=float
                ),
            },
        ],
    )

    joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=["--controller-manager-timeout", "120", "joint_state_broadcaster"],
                condition=IfCondition(LaunchConfiguration("start_joint_state_broadcaster")),
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    imu_sensor_broadcaster = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "--param-file", ros_control_config,
                    "--controller-ros-args", "-r ~/imu:=/imu/data",
                    "imu_sensor_broadcaster",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    foot_fts_broadcasters = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "--param-file", ros_control_config,
                    "FR_foot_fts_broadcaster",
                    "FL_foot_fts_broadcaster",
                    "RR_foot_fts_broadcaster",
                    "RL_foot_fts_broadcaster",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}],
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
        DeclareLaunchArgument("start_joint_state_broadcaster", default_value="false"),
        robot_state_publisher,
        mujoco_ros2_control_node,
        joint_state_broadcaster,
        imu_sensor_broadcaster,
        foot_fts_broadcasters,
        rviz,
    ])
