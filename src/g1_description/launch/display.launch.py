from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    model = LaunchConfiguration("model").perform(context)

    package_share = Path(get_package_share_directory("g1_description"))
    urdf_path = package_share / "urdf" / f"g1_{model}.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")

    return [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(package_share / "launch" / "check_joint.rviz")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value="23dof",
                description="G1 model variant: 23dof or 29dof",
                choices=["23dof", "29dof"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
