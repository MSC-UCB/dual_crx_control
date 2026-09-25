"""Canonical dual-arm bringup with one fixed CRX5IA joint-target interface.

Before use (in each terminal):
    cd /home/msc-crx/ws_fanuc
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash

Usage: ros2 launch dual_crx_control dual_arm.launch.py argument:=value
    # Mock hardware, no RViz, Ruckig interpolation:
    ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false method:=ruckig
    # Real hardware, linear interpolation, targets actually sent at 100 Hz:
    ros2 launch dual_crx_control dual_arm.launch.py mock:=false rviz:=false method:=linear input_rate_hz:=100.0
    # List all arguments:
    ros2 launch dual_crx_control dual_arm.launch.py --show-args

Main arguments and defaults:
    mock:=true; rviz:=true; method:=ruckig (choices: linear / cubic / ruckig).
    input_rate_hz:=50.0: expected target frequency; valid range: 0 < Hz <= 500.
    linear/cubic use 1/input_rate_hz as the transition time; match the actual input rate.
    ruckig_target_mode:=waypoint keeps rest-to-rest targets; stream is explicit opt-in.
    In stream mode input_rate_hz supplies the reference period and the target timeout
    returns to hold after a dropout. Stream mode may add up to one reference period of
    phase lag while preserving the configured velocity, acceleration and jerk limits.
    In waypoint mode Ruckig uses limits, not input_rate_hz, to set arrival time.
    All methods output at 500 Hz; input_rate_hz does not change the sender's frequency.
    left_robot_ip:=192.168.10.100; right_robot_ip:=192.168.10.200.

Input: /crx5ia/joint_targets (JointState; one complete arm or both arms).
Joint names: left_J1..left_J6 / right_J1..right_J6.
This launch starts the control stack. Run a motion script in another terminal, e.g.:
    ros2 run dual_crx_control simple_motion.py --joint 1 --range-deg 1.5 --hold 2 --duration 20 --rate 50
simple_motion.py records its own CSV and generates left/right plots on completion or Ctrl+C.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from dual_crx_control.robot.description import arm_description


def launch_setup(context):
    namespace = "crx5ia"
    prefix = f"/{namespace}"
    mock = LaunchConfiguration("mock").perform(context) == "true"
    xacro_file = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"), "urdf", "dual_crx.urdf.xacro",
    ])
    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    driver_xacro = PathJoinSubstitution([
        FindPackageShare("fanuc_hardware_interface"), "robot", "crx5ia.urdf.xacro",
    ]).perform(context)
    controllers = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"), "config", "dual_arm_controllers.yaml",
    ])
    right_description = arm_description(
        driver_xacro, "right", LaunchConfiguration("right_robot_ip").perform(context), mock)
    left_description = arm_description(
        driver_xacro, "left", LaunchConfiguration("left_robot_ip").perform(context), mock)

    # Driver descriptions own controller interfaces; the combined model owns global TF.
    right_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace=f"{namespace}/right", parameters=[{"robot_description": right_description}],
        remappings=[("/tf", f"{prefix}/right/driver_tf"),
                    ("/tf_static", f"{prefix}/right/driver_tf_static")],
        output="screen",
    )
    left_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace=f"{namespace}/left", parameters=[{"robot_description": left_description}],
        remappings=[("/tf", f"{prefix}/left/driver_tf"),
                    ("/tf_static", f"{prefix}/left/driver_tf_static")],
        output="screen",
    )
    robot1 = Node(
        package="controller_manager", executable="ros2_control_node",
        namespace=f"{namespace}/right", parameters=[controllers],
        remappings=[("robot_description", f"{prefix}/right/robot_description")],
        output="screen",
    )
    robot2 = Node(
        package="controller_manager", executable="ros2_control_node",
        namespace=f"{namespace}/left", parameters=[controllers],
        remappings=[("robot_description", f"{prefix}/left/robot_description")],
        output="screen",
    )
    forward_spawners = [
        Node(
            package="controller_manager", executable="spawner", namespace=f"{namespace}/{side}",
            name="joint_controller_spawner", output="screen",
            arguments=["joint_state_broadcaster", "forward_position_controller",
                       "--controller-manager", f"{prefix}/{side}/controller_manager",
                       "--controller-manager-timeout", "180", "--param-file", controllers],
        )
        for side in ("right", "left")
    ]
    interpolation = Node(
        package="dual_crx_control", executable="interpolation_node",
        namespace=namespace, output="screen", parameters=[{
            "input_rate_hz": ParameterValue(LaunchConfiguration("input_rate_hz"), value_type=float),
            "method": LaunchConfiguration("method"),
            "ruckig_target_mode": LaunchConfiguration("ruckig_target_mode"),
            "ruckig_target_timeout": ParameterValue(
                LaunchConfiguration("ruckig_target_timeout"), value_type=float),
        }],
    )
    joint_state_merger = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        namespace=namespace, name="dual_crx_joint_state_merger", output="screen",
        parameters=[{"robot_description": robot_description,
                     "source_list": [f"{prefix}/left/joint_states", f"{prefix}/right/joint_states"],
                     "rate": 100, "publish_default_positions": False}],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        namespace=namespace, name="dual_crx_robot_state_publisher", output="screen",
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
    return [
        right_state_publisher,
        left_state_publisher,
        robot1,
        robot2,
        *forward_spawners,
        interpolation,
        joint_state_merger,
        robot_state_publisher,
        rviz,
    ]


def generate_launch_description():
    # Resolve mock/IP arguments before expanding driver Xacro descriptions.
    return LaunchDescription([
        DeclareLaunchArgument("mock", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.1.100"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.2.100"),
        DeclareLaunchArgument("input_rate_hz", default_value="50.0"),
        DeclareLaunchArgument("method", default_value="ruckig", choices=["linear", "cubic", "ruckig"]),
        DeclareLaunchArgument("ruckig_target_mode", default_value="stream",
                              choices=["waypoint", "stream"],
                              description="Ruckig target semantics; stream is opt-in"),
        DeclareLaunchArgument("ruckig_target_timeout", default_value="0.2",
                              description="Seconds without a stream target before hold"),
        OpaqueFunction(function=launch_setup),
    ])
