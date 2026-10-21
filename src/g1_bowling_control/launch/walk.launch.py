from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scene",
                default_value="plain",
                description="plain (floor only) or bowling (table, pitch, stumps)",
                choices=["plain", "bowling"],
            ),
            Node(
                package="g1_bowling_control",
                executable="walk_controller.py",
                arguments=["--scene", LaunchConfiguration("scene")],
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
