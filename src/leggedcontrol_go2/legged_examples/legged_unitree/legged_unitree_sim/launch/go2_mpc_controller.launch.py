import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    cheater = LaunchConfiguration("cheater").perform(context)
    controller_name = LaunchConfiguration("controller_name").perform(context)
    start_controller = LaunchConfiguration("start_controller").perform(context)
    start_gait_command = LaunchConfiguration("start_gait_command").perform(context)
    start_command_console = LaunchConfiguration("start_command_console").perform(context)
    start_target_publisher = LaunchConfiguration("start_target_publisher").perform(context)
    contact_sensor_names = LaunchConfiguration("contact_sensor_names").perform(context)
    publish_estimated_base_tf = LaunchConfiguration("publish_estimated_base_tf").perform(context)
    simulation_kp = LaunchConfiguration("simulation_kp").perform(context)
    simulation_kd = LaunchConfiguration("simulation_kd").perform(context)
    use_scheduled_contacts_for_estimator = LaunchConfiguration("use_scheduled_contacts_for_estimator").perform(context)

    controllers_pkg = get_package_share_directory("legged_controllers")
    description_pkg = get_package_share_directory("legged_unitree_description")
    model_path = os.path.join(description_pkg, "urdf", "robot_gz.xacro")
    urdf_file = f"/tmp/legged_control/{robot_type}_mpc.urdf"

    robot_description_writer = Node(
        package="legged_unitree_sim",
        executable="publish_robot_description.py",
        output="screen",
        parameters=[
            {"xacro_file": model_path},
            {"robot_type": robot_type},
            {"output_file": urdf_file},
        ],
    )

    load_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controllers_pkg, "launch", "load_controller.launch.py")),
        launch_arguments={
            "robot_type": robot_type,
            "cheater": cheater,
            "controller_name": controller_name,
            "simulation_mode": "true",
            "simulation_kp": simulation_kp,
            "simulation_kd": simulation_kd,
            "use_scheduled_contacts_for_estimator": use_scheduled_contacts_for_estimator,
            "start_controller": start_controller,
            "start_joint_state_broadcaster": "false",
            "start_gait_command": start_gait_command,
            "start_command_console": start_command_console,
            "start_target_publisher": start_target_publisher,
            "urdf_file": urdf_file,
            "contact_sensor_names": contact_sensor_names,
            "publish_estimated_base_tf": publish_estimated_base_tf,
        }.items(),
    )

    return [
        robot_description_writer,
        RegisterEventHandler(OnProcessExit(target_action=robot_description_writer, on_exit=[load_controller])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="go2"),
        DeclareLaunchArgument("cheater", default_value="true"),
        DeclareLaunchArgument("controller_name", default_value=""),
        DeclareLaunchArgument("start_controller", default_value="false"),
        DeclareLaunchArgument("start_gait_command", default_value="true"),
        DeclareLaunchArgument("start_command_console", default_value="true"),
        DeclareLaunchArgument("start_target_publisher", default_value="true"),
        # Empty selects boolean LF/RF/LH/RH contact interfaces (Gazebo).
        # MuJoCo overrides this with its four F/T sensor names.
        DeclareLaunchArgument("contact_sensor_names", default_value=""),
        DeclareLaunchArgument("publish_estimated_base_tf", default_value="true"),
        DeclareLaunchArgument("simulation_kp", default_value="0.0"),
        DeclareLaunchArgument("simulation_kd", default_value="3.0"),
        DeclareLaunchArgument("use_scheduled_contacts_for_estimator", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
