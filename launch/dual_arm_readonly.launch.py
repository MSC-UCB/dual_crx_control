from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():

    namespace = "crx5ia"

    # --------------------------------------------------
    # Combined dual-arm URDF
    # --------------------------------------------------

    xacro_file = PathJoinSubstitution([
        FindPackageShare("dual_crx_control"),
        "urdf",
        "dual_crx.urdf.xacro",
    ])

    robot_description = Command([
        "xacro ",
        xacro_file,
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="dual_crx_robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        output="screen",
    )

    # --------------------------------------------------
    # FANUC physical driver
    # --------------------------------------------------

    fanuc_physical_launch = PathJoinSubstitution([
        FindPackageShare("fanuc_hardware_interface"),
        "launch",
        "fanuc_physical_control.launch.py",
    ])

    # Right CRX
    right_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fanuc_physical_launch),
        launch_arguments={
            "robot_ip": "192.168.10.200",
            "robot_series": "crx",
            "robot_model": "crx5ia",

            "namespace": "crx5ia/right",
            "prefix": "right_",

            "use_mock": "false",
            "launch_rviz": "false",

            # Important:
            # Keep motion authority on FANUC controller
            "motion_control": "0",

            # Do not start a motion controller
            "initial_controller": "none",
        }.items(),
    )

    # Left CRX
    left_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fanuc_physical_launch),
        launch_arguments={
            "robot_ip": "192.168.10.100",
            "robot_series": "crx",
            "robot_model": "crx5ia",

            "namespace": "crx5ia/left",
            "prefix": "left_",

            "use_mock": "false",
            "launch_rviz": "false",

            # Important:
            # Keep motion authority on FANUC controller
            "motion_control": "0",

            # Do not start a motion controller
            "initial_controller": "none",
        }.items(),
    )

    return LaunchDescription([
        right_robot,
        left_robot,
        robot_state_publisher,
        rviz,
    ])
