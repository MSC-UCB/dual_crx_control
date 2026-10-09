"""Single CRX bringup. Select prefix:=left_ or right_; mock hardware is the default.

control:=none starts only the stack; cartesian starts a sine motion, and
admittance opens the two-panel GUI with the inactive arm disabled.
The selected arm's world model preserves base calibration and TCP definitions.
"""
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from dual_crx_control.robot.description import arm_description


def launch_setup(context):
    side = LaunchConfiguration("prefix").perform(context).removesuffix("_")
    if side not in ("left", "right"):
        raise ValueError("prefix must be left_ or right_")
    active_arms = [side]
    control = LaunchConfiguration("control").perform(context)
    namespace = "crx5ia"
    prefix = f"/{namespace}"
    mock = LaunchConfiguration("mock").perform(context) == "true"
    read_only = LaunchConfiguration("read_only").perform(context) == "true"
    collision_force_limit = LaunchConfiguration("collision_force_limit").perform(context) == "true"
    if read_only and control != "none":
        raise ValueError("read_only requires control:=none")
    if control == "admittance" and not collision_force_limit:
        raise ValueError("admittance requires collision_force_limit:=true")
    xacro_file = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"), "urdf", "dual_crx.urdf.xacro",
    ])
    robot_description = ParameterValue(Command([
        "xacro ", xacro_file,
        " enable_left:=", str(side == "left").lower(),
        " enable_right:=", str(side == "right").lower(),
    ]), value_type=str)
    driver_xacro = PathJoinSubstitution([
        FindPackageShare("fanuc_hardware_interface"), "robot", "crx5ia.urdf.xacro",
    ]).perform(context)
    controllers = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"), "config", "dual_arm_controllers.yaml",
    ])
    description = arm_description(
        driver_xacro, side, LaunchConfiguration(f"{side}_robot_ip").perform(context), mock, read_only)
    if mock:
        # GenericSystem otherwise exposes NaN sensor states. A stationary mock
        # starts with zero wrench; physical sensor values are never overridden.
        root = ET.fromstring(description)
        for interface in root.findall("ros2_control/sensor[@name='ft_sensor']/state_interface"):
            initial = interface.find("param[@name='initial_value']")
            if initial is None:
                initial = ET.SubElement(interface, "param", name="initial_value")
            initial.text = "0.0"
        description = ET.tostring(root, encoding="unicode")
    driver_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace=f"{namespace}/{side}", parameters=[{"robot_description": description}],
        remappings=[("/tf", f"{prefix}/{side}/driver_tf"),
                    ("/tf_static", f"{prefix}/{side}/driver_tf_static")],
        output="screen",
    )
    controller_manager = Node(
        package="controller_manager", executable="ros2_control_node",
        namespace=f"{namespace}/{side}", parameters=[controllers],
        remappings=[("robot_description", f"{prefix}/{side}/robot_description")],
        output="screen",
    )
    forward_spawners = [
        Node(
            package="controller_manager", executable="spawner", namespace=f"{namespace}/{side}",
            name="joint_controller_spawner", output="screen",
            arguments=["joint_state_broadcaster", "force_torque_sensor_broadcaster",
                       *([] if read_only else ["forward_position_controller"]),
                       "--controller-manager", f"{prefix}/{side}/controller_manager",
                       "--controller-manager-timeout", "180", "--param-file", controllers],
        )
        for side in active_arms
    ]
    interpolation = Node(
        package="dual_crx_control", executable="interpolation_node",
        namespace=namespace, output="screen", parameters=[{
            "active_arms": active_arms,
            "input_rate_hz": ParameterValue(LaunchConfiguration("input_rate_hz"), value_type=float),
            "method": LaunchConfiguration("method"),
            "ruckig_target_mode": LaunchConfiguration("ruckig_target_mode"),
            "ruckig_target_timeout": ParameterValue(
                LaunchConfiguration("ruckig_target_timeout"), value_type=float),
        }],
    )
    limiter_actions = []
    if collision_force_limit and not read_only:
        limiter = Node(
            package="dual_crx_control", executable="collision_force_limiter.py",
            namespace=namespace, output="screen", parameters=[{
                "active_arms": active_arms,
                "collision_force_threshold_n": ParameterValue(
                    LaunchConfiguration("collision_force_threshold_n"), value_type=float),
            }], respawn=False,
        )
        limiter_actions = [
            RegisterEventHandler(OnProcessExit(
                target_action=limiter, on_exit=[EmitEvent(event=Shutdown(
                    reason="Collision force limiter exited; stopping control launch"))])),
            limiter,
        ]
    else:
        limiter_actions = [LogInfo(msg=(
            "Collision force limiter disabled: read_only mode" if read_only else
            "Collision force limiter disabled: collision_force_limit=false"))]
    joint_state_merger = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        namespace=namespace, name="single_crx_joint_state_merger", output="screen",
        # The combined model makes joint_state_publisher pad missing joints with
        # zeros even with publish_default_positions=false. Use only the driver's
        # six joints here; the world state publisher below keeps calibrated TF.
        remappings=[("robot_description", f"{prefix}/{side}/robot_description")],
        parameters=[{"source_list": [f"{prefix}/{side}/joint_states"],
                     "rate": 100, "publish_default_positions": False}],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace=namespace, name="single_crx_robot_state_publisher", output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", namespace=namespace, output="screen",
        remappings=[("/robot_description", f"{prefix}/robot_description")],
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("dual_crx_control"), "rviz", "dual_cartesian.rviz",
        ])],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    motion = []
    if control != "none":
        parameters = {"active_arms": active_arms}
        if control == "cartesian":
            parameters.update(
                rate=ParameterValue(LaunchConfiguration("input_rate_hz"), value_type=float),
                move_to_initial=ParameterValue(LaunchConfiguration("move_to_initial"), value_type=bool))
        motion.append(Node(
            package="dual_crx_control", namespace=namespace, output="screen",
            executable=("cartesian_sine.py" if control == "cartesian" else "admittance_hand_guiding.py"),
            parameters=[parameters],
        ))
    return [
        driver_state_publisher,
        controller_manager,
        *forward_spawners,
        *([] if read_only else [interpolation]),
        *limiter_actions,
        joint_state_merger,
        robot_state_publisher,
        rviz,
        *motion,
    ]


def generate_launch_description():
    # Resolve mock/IP arguments before expanding driver Xacro descriptions.
    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value="left_", choices=["left_", "right_"],
                              description="Joint prefix selecting the only active arm"),
        DeclareLaunchArgument("control", default_value="none", choices=["none", "cartesian", "admittance"],
                              description="Optional Cartesian sine motion or admittance GUI"),
        DeclareLaunchArgument("move_to_initial", default_value="false", choices=["true", "false"],
                              description="Approach initial_pose.yaml before optional Cartesian motion"),
        DeclareLaunchArgument("mock", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("read_only", default_value="false", choices=["true", "false"],
                              description="Disable motion control, command controller and interpolation"),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("collision_force_limit", default_value="true", choices=["true", "false"],
                              description="Latch a selected-arm stop on excessive force; disabled in read-only mode"),
        DeclareLaunchArgument("collision_force_threshold_n", default_value="20.0",
                              description="Stop the selected arm when any absolute force component exceeds this many N"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.2.100"),
        DeclareLaunchArgument("input_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("method", default_value="quintic", choices=["linear", "cubic", "ruckig", "quintic"]),
        DeclareLaunchArgument("ruckig_target_mode", default_value="stream",
                              choices=["waypoint", "stream"],
                              description="Ruckig target semantics"),
        DeclareLaunchArgument("ruckig_target_timeout", default_value="0.2",
                              description="Seconds without a stream target before hold"),
        OpaqueFunction(function=launch_setup),
    ])
