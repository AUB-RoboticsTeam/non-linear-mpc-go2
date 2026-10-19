import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")

    ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control_mujoco.yaml"
    )
    mpc_ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control_mpc_mujoco.yaml"
    )

    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_mujoco_scene_path = os.path.join(
        unitree_go2_description, "mujoco/go2/scene.xml"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (MuJoCo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_controller_mode = DeclareLaunchArgument(
        "controller_mode",
        default_value="unitree",
        description="Controller stack to spawn: unitree or mpc",
    )
    declare_wbc_type = DeclareLaunchArgument(
        "wbc_type",
        default_value="weighted",
        description="MPC WBC backend: weighted or hoq",
    )
    declare_mpc_start_controller = DeclareLaunchArgument(
        "mpc_start_controller",
        default_value="true",
        description="Start the MPC legged controller active. Set false to spawn it inactive for debugging.",
    )
    selected_ros_control_file = PythonExpression([
        "'", mpc_ros_control_config, "' if '", LaunchConfiguration("controller_mode"), "' == 'mpc' else '", ros_control_config, "'"
    ])
    is_mpc_mode = PythonExpression(["'", LaunchConfiguration("controller_mode"), "' == 'mpc'"])
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=selected_ros_control_file,
        description="Ros control config path",
    )
    declare_mujoco_model = DeclareLaunchArgument(
        "mujoco_model_path",
        default_value=default_mujoco_scene_path,
        description="Path to the MuJoCo MJCF scene file",
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )

    robot_description = {
        "robot_description": Command([
            "xacro ", LaunchConfiguration("unitree_go2_description_path"), " sim:=mujoco"
        ])
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(unitree_go2_sim, "rviz/rviz.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    legged_controllers_pkg = get_package_share_directory("legged_controllers")
    mpc_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(legged_controllers_pkg, "launch", "load_controller.launch.py")),
        condition=IfCondition(is_mpc_mode),
        launch_arguments={
            "robot_type": "go2",
            "simulation_mode": "true",
            "start_controller": LaunchConfiguration("mpc_start_controller"),
            "start_joint_state_broadcaster": "false",
            "start_gait_command": "true",
            "start_target_publisher": "true",
            "joints": "lf_hip_joint,lf_upper_leg_joint,lf_lower_leg_joint,rf_hip_joint,rf_upper_leg_joint,rf_lower_leg_joint,lh_hip_joint,lh_upper_leg_joint,lh_lower_leg_joint,rh_hip_joint,rh_upper_leg_joint,rh_lower_leg_joint",
            "imu_name": "trunk_imu",
            "contact_sensor_names": "FL_foot_fts,FR_foot_fts,RL_foot_fts,RR_foot_fts",
            "contact_force_threshold": "20.0",
            "wbc_type": LaunchConfiguration("wbc_type"),
        }.items(),
    )

    # mujoco_ros2_control both steps physics and hosts the controller_manager;
    # unlike gz_ros2_control there is no separate "spawn the robot" step and no
    # headless flag today, so the MuJoCo GLFW viewer window always opens.
    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            LaunchConfiguration("ros_control_file"),
            {"mujoco_model_path": LaunchConfiguration("mujoco_model_path")},
        ],
    )

    controller_spawner_js = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "joint_states_controller",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    controller_spawner_imu = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    # controller_manager builds each spawned controller's node
                    # with use_global_arguments(false), so params/remaps
                    # passed only to the mujoco_ros2_control process never
                    # reach it; --param-file/--controller-ros-args are the
                    # mechanisms controller_manager actually threads through
                    # per-controller (see determine_controller_node_options()
                    # in controller_manager.cpp).
                    "--param-file", LaunchConfiguration("ros_control_file"),
                    # imu_sensor_broadcaster publishes on its default relative
                    # topic; remap it to /imu/data to match the topic
                    # convention used by the Gazebo backend.
                    "--controller-ros-args", "-r ~/imu:=/imu/data",
                    "imu_sensor_broadcaster",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    controller_spawner_foot_fts = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                arguments=[
                    "--controller-manager-timeout", "120",
                    "--param-file", LaunchConfiguration("ros_control_file"),
                    "FR_foot_fts_broadcaster",
                    "FL_foot_fts_broadcaster",
                    "RR_foot_fts_broadcaster",
                    "RL_foot_fts_broadcaster",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )

    controller_spawner_unitree = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                output="screen",
                condition=UnlessCondition(is_mpc_mode),
                arguments=[
                    "--controller-manager-timeout", "120",
                    "--param-file", LaunchConfiguration("ros_control_file"),

                    "FR_hip_controller",
                    "FR_thigh_controller",
                    "FR_calf_controller",

                    "FL_hip_controller",
                    "FL_thigh_controller",
                    "FL_calf_controller",

                    "RR_hip_controller",
                    "RR_thigh_controller",
                    "RR_calf_controller",

                    "RL_hip_controller",
                    "RL_thigh_controller",
                    "RL_calf_controller",
                ],
                parameters=[{"use_sim_time": use_sim_time},
                robot_description
                ],
            )
        ]
    )

    controller_status_check = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=["bash", "-c", "echo 'Checking controller status:' && ros2 control list_controllers"],
                output='screen',
            )
        ]
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_controller_mode,
            declare_wbc_type,
            declare_mpc_start_controller,
            declare_ros_control_file,
            declare_mujoco_model,
            declare_description_path,

            mujoco_ros2_control_node,
            robot_state_publisher_node,

            controller_spawner_js,
            controller_spawner_imu,
            controller_spawner_foot_fts,
            controller_spawner_unitree,
            mpc_controller,
            controller_status_check,

            rviz2,
        ]
    )
