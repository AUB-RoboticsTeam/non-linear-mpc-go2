import os
import time

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # gz-transport defaults GZ_PARTITION to "<hostname>:<user>", so every gz sim
    # process a user runs shares one transport bus. If a previous run's "gz sim
    # server"/"gz sim gui" didn't fully exit (e.g. killed abruptly instead of via
    # SIGINT), a new launch can discover/collide with that stale process on the
    # same bus, which shows up as flaky/missing robot spawns or a controller
    # spawner that hangs waiting on a service served by the dead run. Giving each
    # launch its own partition makes stale processes from earlier runs invisible
    # to this one, so leftovers can no longer corrupt a fresh launch.
    set_gz_partition = SetEnvironmentVariable(
        "GZ_PARTITION", f"unitree_go2_sim_{os.getpid()}_{int(time.time())}"
    )

    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim").find("unitree_go2_sim")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description").find("unitree_go2_description")

    ros_control_config = os.path.join(
        unitree_go2_sim, "config/ros_control/ros_control.yaml"
    )

    default_model_path = os.path.join(unitree_go2_description, "urdf/unitree_go2_robot.xacro")
    default_world_path = os.path.join(unitree_go2_description, "worlds/elevation.sdf")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="2.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="2.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.5")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )
    declare_description_path = DeclareLaunchArgument(
        "unitree_go2_description_path",
        default_value=default_model_path,
        description="Path to the robot description xacro file",
    )

    robot_description = {
        "robot_description": Command(["xacro ", LaunchConfiguration("unitree_go2_description_path")])
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

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                unitree_go2_description,
                'worlds',
                'elevation.sdf'
            ]), ' -r',
            # "gui" was previously declared but never actually applied, so gz sim
            # always launched with its GUI/rendering client even when gui:=false
            # was passed. That doubled CPU/GPU load on every "headless" run and
            # made the fixed startup timeouts elsewhere in this launch (spawn,
            # controller loading) more likely to race and fail under load.
            # "-s" runs the server only, matching what gui:=false is meant to do.
            PythonExpression(["'' if '", LaunchConfiguration('gui'), "' == 'true' else ' -s'"])]
        }.items(),
    )

    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', LaunchConfiguration('world_init_z'),
            '-Y', LaunchConfiguration('world_init_heading')
        ],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/velodyne_points/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/unitree_lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/rgb_image@sensor_msgs/msg/Image@gz.msgs.Image',

            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_group_effort_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
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

    controller_spawner_unitree = TimerAction(
    period=5.0,
    actions=[
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "--controller-manager-timeout", "120",

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
            set_gz_partition,

            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_description_path,

            gz_sim,
            robot_state_publisher_node,
            gazebo_spawn_robot,
            gazebo_bridge,

            #map_to_odom_tf_node,
            #base_footprint_to_base_link_tf_node,

            controller_spawner_js,
            controller_spawner_unitree,
            controller_status_check,

            rviz2,
        ]
    )
