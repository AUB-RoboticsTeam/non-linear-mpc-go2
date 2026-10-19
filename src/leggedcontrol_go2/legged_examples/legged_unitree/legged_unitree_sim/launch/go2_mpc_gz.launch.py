import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("legged_unitree_sim")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "go2_gz.launch.py")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_type": LaunchConfiguration("robot_type"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "rviz": LaunchConfiguration("rviz"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "yaw": LaunchConfiguration("yaw"),
            "start_joint_state_broadcaster": "true",
            "start_sensor_broadcasters": "true",
        }.items(),
    )

    mpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "go2_mpc_controller.launch.py")),
        launch_arguments={
            "robot_type": LaunchConfiguration("robot_type"),
            "cheater": LaunchConfiguration("cheater"),
            # Keep the public controller name identical to MuJoCo so the GUI,
            # joystick and documented switch command work on either backend.
            "controller_name": "legged_controller",
            "start_controller": LaunchConfiguration("start_controller"),
            "start_gait_command": LaunchConfiguration("start_gait_command"),
            "start_command_console": LaunchConfiguration("start_command_console"),
            "start_target_publisher": LaunchConfiguration("start_target_publisher"),
            # Keep the same F/T sensor ordering used by the MuJoCo backend:
            # LF, RF, LH, RH in OCS2 contact order.
            "contact_sensor_names": "FL_foot_fts,FR_foot_fts,RL_foot_fts,RR_foot_fts",
            "publish_estimated_base_tf": "false",
            "simulation_kp": LaunchConfiguration("simulation_kp"),
            "simulation_kd": LaunchConfiguration("simulation_kd"),
            "use_scheduled_contacts_for_estimator": "true",
        }.items(),
    )

    command_gui = Node(
        package="legged_controllers",
        executable="legged_command_gui.py",
        name="legged_command_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_command_gui")),
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "simulation_backend": "gazebo",
            "gazebo_robot_name": LaunchConfiguration("robot_name"),
            "gazebo_activation_x": LaunchConfiguration("x"),
            "gazebo_activation_y": LaunchConfiguration("y"),
            "gazebo_activation_z": LaunchConfiguration("stance_height"),
            "gazebo_activation_yaw": LaunchConfiguration("yaw"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot_type", default_value="go2"),
        DeclareLaunchArgument("robot_name", default_value="go2"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(
                get_package_share_directory("legged_unitree_description"), "worlds", "default.sdf"
            ),
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.42"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        # Gazebo's contact transitions make the leg-kinematic Kalman estimate
        # diverge during diagonal support, even though its IMU and force data
        # remain useful and are still published.  Use exact simulated base
        # odometry for the stable Gazebo MPC pipeline.  Keep cheater:=false as
        # an explicit estimator-development option.  This is Gazebo-only;
        # MuJoCo and real hardware retain their existing estimators.
        DeclareLaunchArgument("cheater", default_value="true"),
        DeclareLaunchArgument("start_controller", default_value="false"),
        DeclareLaunchArgument("start_gait_command", default_value="true"),
        DeclareLaunchArgument("start_command_console", default_value="false"),
        # Keep Gazebo and the operator UI in separate terminals by default.
        # The standalone GUI auto-detects Gazebo and its STAND button performs
        # the safe reset / hold / controller activation handoff.
        DeclareLaunchArgument("start_command_gui", default_value="false"),
        DeclareLaunchArgument("stance_height", default_value="0.34"),
        # Gazebo uses effort-only joints and differs from MuJoCo in solver and
        # contact compliance. These gains track the WBC joint trajectory while
        # retaining its optimized feed-forward torques.
        DeclareLaunchArgument("simulation_kp", default_value="12.0"),
        DeclareLaunchArgument("simulation_kd", default_value="3.0"),
        DeclareLaunchArgument("start_target_publisher", default_value="true"),
        DeclareLaunchArgument("mpc_start_delay", default_value="8.0"),
        gz_launch,
        TimerAction(period=LaunchConfiguration("mpc_start_delay"), actions=[mpc_launch]),
        command_gui,
    ])
