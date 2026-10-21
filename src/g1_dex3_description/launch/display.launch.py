from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    viewer = LaunchConfiguration("viewer").perform(context)
    fixed_base = LaunchConfiguration("fixed_base").perform(context) == "true"
    scene = LaunchConfiguration("scene").perform(context)

    package_share = Path(get_package_share_directory("g1_dex3_description"))

    if viewer == "mujoco":
        scene_file = "bowling_scene.xml" if scene == "bowling" else "scene.xml"
        arguments = ["--scene", str(package_share / "mjcf" / scene_file)]
        if fixed_base:
            arguments.append("--fixed-base")
        return [
            Node(
                package="g1_dex3_description",
                executable="mujoco_viewer.py",
                arguments=arguments,
                output="screen",
            ),
        ]

    robot_description = (package_share / "urdf" / "g1_29dof_dex3.urdf").read_text(
        encoding="utf-8")

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
            arguments=["-d", str(package_share / "rviz" / "g1_dex3.rviz")],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "viewer",
                default_value="rviz",
                description="Where to display the G1 + Dex3-1: rviz or mujoco",
                choices=["rviz", "mujoco"],
            ),
            DeclareLaunchArgument(
                "scene",
                default_value="bowling",
                description="MuJoCo only: plain (floor only) or bowling (table, tray, ball, pitch, stumps)",
                choices=["plain", "bowling"],
            ),
            DeclareLaunchArgument(
                "fixed_base",
                default_value="true",
                description="MuJoCo only: weld the pelvis so the robot does not fall",
                choices=["true", "false"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
