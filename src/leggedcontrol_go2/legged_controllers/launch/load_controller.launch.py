from pathlib import Path
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


JOINTS = [
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LH_HAA", "LH_HFE", "LH_KFE",
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RH_HAA", "RH_HFE", "RH_KFE",
]


def _bool_arg(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true", "yes", "on")


def _write_controller_params_file(robot_type: str, urdf_file: str, task_file: str, reference_file: str, cheater: bool,
                                  simulation_mode: bool, joints: list[str], imu_name: str,
                                  contact_sensor_names: list[str], contact_force_threshold: float, wbc_type: str,
                                  publish_estimated_base_tf: bool, simulation_kp: float, simulation_kd: float,
                                  use_scheduled_contacts_for_estimator: bool, enable_visualization: bool,
                                  publish_mpc_observation: bool, mpc_observation_publish_rate: float,
                                  controller_name: str) -> str:
    controller_dir = Path("/tmp/legged_control")
    controller_dir.mkdir(parents=True, exist_ok=True)
    params_file = controller_dir / f"{robot_type}_legged_controller.yaml"
    quoted_joints = ", ".join([f"'{joint}'" for joint in joints])
    quoted_contacts = ", ".join([f"'{name}'" for name in contact_sensor_names])
    contact_parameter = (
        f"    contact_sensor_names: [{quoted_contacts}]\n"
        if contact_sensor_names
        else ""
    )
    params_file.write_text(f"""{controller_name}:
  ros__parameters:
    use_sim_time: {str(simulation_mode).lower()}
    urdfFile: {urdf_file}
    taskFile: {task_file}
    referenceFile: {reference_file}
    joints: [{quoted_joints}]
    imu_name: {imu_name}
{contact_parameter}\
    contact_force_threshold: {contact_force_threshold}
    wbc_type: {wbc_type}
    cheater: {str(cheater).lower()}
    simulation_mode: {str(simulation_mode).lower()}
    simulation_kp: {simulation_kp}
    simulation_kd: {simulation_kd}
    use_scheduled_contacts_for_estimator: {str(use_scheduled_contacts_for_estimator).lower()}
    publish_estimated_base_tf: {str(publish_estimated_base_tf).lower()}
    enable_visualization: {str(enable_visualization).lower()}
    publish_mpc_observation: {str(publish_mpc_observation).lower()}
    mpc_observation_publish_rate: {mpc_observation_publish_rate}
""")
    return str(params_file)


def _split_csv(value: str) -> list[str]:
    return [item.strip() for item in value.split(",") if item.strip()]


def _launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    cheater = _bool_arg(context, "cheater")
    simulation_mode = _bool_arg(context, "simulation_mode")
    start_controller = _bool_arg(context, "start_controller")
    start_joint_state_broadcaster = _bool_arg(context, "start_joint_state_broadcaster")
    start_gait_command = _bool_arg(context, "start_gait_command")
    start_command_console = _bool_arg(context, "start_command_console")
    start_target_publisher = _bool_arg(context, "start_target_publisher")
    publish_estimated_base_tf = _bool_arg(context, "publish_estimated_base_tf")
    simulation_kp = float(LaunchConfiguration("simulation_kp").perform(context))
    simulation_kd = float(LaunchConfiguration("simulation_kd").perform(context))
    use_scheduled_contacts_for_estimator = _bool_arg(context, "use_scheduled_contacts_for_estimator")
    enable_visualization = _bool_arg(context, "enable_visualization")
    publish_mpc_observation = _bool_arg(context, "publish_mpc_observation")
    mpc_observation_publish_rate = float(LaunchConfiguration("mpc_observation_publish_rate").perform(context))
    controller_name = LaunchConfiguration("controller_name").perform(context).strip()
    if not controller_name:
        controller_name = "legged_cheater_controller" if cheater else "legged_controller"

    controllers_pkg = get_package_share_directory("legged_controllers")
    task_file = os.path.join(controllers_pkg, "config", robot_type, "task.info")
    reference_file = os.path.join(controllers_pkg, "config", robot_type, "reference.info")
    gait_command_file = os.path.join(controllers_pkg, "config", robot_type, "gait.info")
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    if not urdf_file:
        controller_dir = Path("/tmp/legged_control")
        controller_dir.mkdir(parents=True, exist_ok=True)
        urdf_file = str(controller_dir / f"{robot_type}.urdf")
        description_pkg = get_package_share_directory("legged_unitree_description")
        xacro_file = os.path.join(description_pkg, "urdf", "robot.xacro")
        subprocess.run(["xacro", xacro_file, f"robot_type:={robot_type}", "-o", urdf_file], check=True)
    joints = _split_csv(LaunchConfiguration("joints").perform(context)) or JOINTS
    imu_name = LaunchConfiguration("imu_name").perform(context)
    contact_sensor_names = _split_csv(LaunchConfiguration("contact_sensor_names").perform(context))
    contact_force_threshold = float(LaunchConfiguration("contact_force_threshold").perform(context))
    wbc_type = LaunchConfiguration("wbc_type").perform(context)
    controller_params_file = _write_controller_params_file(robot_type, urdf_file, task_file, reference_file, cheater, simulation_mode,
                                                           joints, imu_name, contact_sensor_names, contact_force_threshold, wbc_type,
                                                           publish_estimated_base_tf, simulation_kp, simulation_kd,
                                                           use_scheduled_contacts_for_estimator, enable_visualization,
                                                           publish_mpc_observation, mpc_observation_publish_rate,
                                                           controller_name)

    # Full-centroidal CppAD models are generated on the first run and can take
    # longer than the spawner's 10 s service-response default.  Subsequent
    # launches reuse the generated libraries, but the first launch must also
    # complete deterministically.
    spawner_args = [
        controller_name,
        "-p",
        controller_params_file,
        "--controller-manager-timeout",
        "120",
        "--service-call-timeout",
        "180",
        "--switch-timeout",
        "120",
    ]
    if not start_controller:
        spawner_args.append("--inactive")

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": simulation_mode}],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=spawner_args,
        parameters=[{"use_sim_time": simulation_mode}],
    )

    if start_joint_state_broadcaster:
        nodes = [
            joint_state_broadcaster,
            RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[controller_spawner])),
        ]
    else:
        nodes = [controller_spawner]

    if start_gait_command:
        nodes.append(Node(
            package="legged_controllers",
            executable="legged_gait_command_bridge.py",
            name="legged_gait_command_bridge",
            output="screen",
            parameters=[
                {"gaitCommandFile": gait_command_file},
                {"robotName": "legged_robot"},
                {"use_sim_time": simulation_mode},
            ],
        ))

    if start_target_publisher:
        nodes.append(Node(
            package="legged_controllers",
            executable="legged_target_trajectories_publisher",
            name="legged_robot_target",
            output="screen",
            parameters=[
                {"taskFile": task_file},
                {"referenceFile": reference_file},
                {"use_sim_time": simulation_mode},
            ],
        ))

    if start_command_console:
        nodes.append(ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                "ros2 run legged_controllers legged_cheater_console.py "
                f"--ros-args -p controller_name:={controller_name} < /dev/tty",
            ],
            output="screen",
            emulate_tty=True,
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_type", default_value="go2"),
        DeclareLaunchArgument("cheater", default_value="false"),
        # Optional stable ROS controller name independent of estimator type.
        # An empty value preserves the historical normal/cheater names.
        DeclareLaunchArgument("controller_name", default_value=""),
        # true by default because both the Gazebo Sim and MuJoCo ros2_control
        # backends only support standard effort/position/velocity command
        # interfaces, not this controller's hybrid position/velocity/kp/kd/
        # feedforward interface (see legged_common's HybridJointInterface) --
        # unlike ROS1, where legged_gazebo's RobotHWSim spoke the hybrid
        # interface natively and no such flag was needed. Real hardware
        # (legged_unitree_hw) does support the hybrid interface, so hardware
        # launches must pass simulation_mode:=false explicitly.
        DeclareLaunchArgument("simulation_mode", default_value="true"),
        # Defaults preserve the working MuJoCo behavior. Gazebo overrides
        # these because its effort-only joint/contact dynamics need modest
        # position tracking in addition to WBC feed-forward torque.
        DeclareLaunchArgument("simulation_kp", default_value="0.0"),
        DeclareLaunchArgument("simulation_kd", default_value="3.0"),
        DeclareLaunchArgument("use_scheduled_contacts_for_estimator", default_value="false"),
        DeclareLaunchArgument("publish_estimated_base_tf", default_value="true"),
        DeclareLaunchArgument("enable_visualization", default_value="true"),
        DeclareLaunchArgument("publish_mpc_observation", default_value="true"),
        DeclareLaunchArgument("mpc_observation_publish_rate", default_value="500.0"),
        # false by default to match ROS1's `load_controller.launch`, which
        # calls `controller_manager` with the `load` verb: this LOADS the
        # controller without starting it. In the ROS1 workflow, starting it
        # is a separate, explicit step -- a `rosservice call
        # /controller_manager/switch_controller` (see legged_controllers'
        # README/quick-start). Here that's `ros2 service call
        # /controller_manager/switch_controller
        # controller_manager_msgs/srv/SwitchController "{activate_controllers:
        # ['legged_controller'], strictness: 2}"` (ROS2's spawner tool would
        # normally load *and* start, like ROS1's `spawner`/`controller_manager
        # spawn` -- but we deliberately keep the two steps separate here so
        # the activation step is visible and debuggable on its own, matching
        # ROS1's staged behavior). Pass start_controller:=true to spawn
        # active immediately instead.
        DeclareLaunchArgument("start_controller", default_value="false"),
        DeclareLaunchArgument("start_joint_state_broadcaster", default_value="true"),
        DeclareLaunchArgument("start_gait_command", default_value="true"),
        DeclareLaunchArgument("start_command_console", default_value="false"),
        DeclareLaunchArgument("start_target_publisher", default_value="true"),
        DeclareLaunchArgument("urdf_file", default_value=""),
        DeclareLaunchArgument("joints", default_value=""),
        DeclareLaunchArgument("imu_name", default_value="unitree_imu"),
        # Empty uses the OCS2 boolean foot contact interfaces. MuJoCo passes
        # its force/torque sensor names explicitly.
        DeclareLaunchArgument("contact_sensor_names", default_value=""),
        DeclareLaunchArgument("contact_force_threshold", default_value="20.0"),
        DeclareLaunchArgument("wbc_type", default_value="weighted"),
        OpaqueFunction(function=_launch_setup),
    ])
