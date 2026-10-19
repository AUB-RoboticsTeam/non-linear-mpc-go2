import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_type = LaunchConfiguration("robot_type")
    robot_name = LaunchConfiguration("robot_name")
    world = LaunchConfiguration("world")

    description_pkg = FindPackageShare("legged_unitree_description")
    model_path = PathJoinSubstitution([description_pkg, "urdf", "robot_gz.xacro"])
    default_world = PathJoinSubstitution([description_pkg, "worlds", "default.sdf"])
    ros_control_config = os.path.join(
        get_package_share_directory("legged_unitree_sim"), "config", "gz_ros2_control.yaml"
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            model_path,
            " robot_type:=",
            robot_type,
        ])
    }

    robot_description_publisher = Node(
        package="legged_unitree_sim",
        executable="publish_robot_description.py",
        output="screen",
        parameters=[
            {"xacro_file": model_path},
            {"robot_type": robot_type},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        # Gazebo otherwise opens paused.  Start physics immediately and leave
        # the legged controller inactive, which gives the requested passive /
        # joint-damped startup state until the GUI's STAND workflow is used.
        launch_arguments={"gz_args": [world, " -r"]}.items(),
    )

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
    )

    spawn_robot = TimerAction(
        period=1.2,
        actions=[spawn_robot_node],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Sensor data must flow only Gazebo -> ROS. Bidirectional bridges
            # feed broadcaster output back into the simulator transport topic.
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/ground_truth/state@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/feet/LF/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/feet/RL/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/feet/RF/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
            "/feet/RR/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
        ],
        # Keep raw Gazebo IMU samples separate from the public broadcaster
        # output at /imu/data, otherwise ImuStateSensor subscribes to itself.
        remappings=[("/imu/data", "/gz/imu/data")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "--controller-manager-timeout", "120", "--inactive",
                    "joint_state_broadcaster",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_joint_state_broadcaster")),
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    imu_sensor_broadcaster = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "--controller-manager-timeout", "120", "--inactive",
                    "--param-file", ros_control_config,
                    "--controller-ros-args", "-r ~/imu:=/imu/data",
                    "imu_sensor_broadcaster",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_sensor_broadcasters")),
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    foot_fts_broadcasters = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "--controller-manager-timeout", "120", "--inactive",
                    "--param-file", ros_control_config,
                    "FR_foot_fts_broadcaster", "FL_foot_fts_broadcaster",
                    "RR_foot_fts_broadcaster", "RL_foot_fts_broadcaster",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_sensor_broadcasters")),
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot_type", default_value="go2"),
        DeclareLaunchArgument("robot_name", default_value="go2"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("start_joint_state_broadcaster", default_value="false"),
        DeclareLaunchArgument("start_sensor_broadcasters", default_value="false"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.34"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        robot_description_publisher,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        joint_state_broadcaster,
        imu_sensor_broadcaster,
        foot_fts_broadcasters,
        rviz,
    ])
