from ament_index_python.packages import get_package_share_directory
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    hand = LaunchConfiguration("hand").perform(context)
    side = hand[0]  # "l" or "r"

    package_share = Path(get_package_share_directory("dex3_description"))
    urdf_path = package_share / "dex3_1" / f"dex3_1_{side}.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")

    # world -> palm is a floating joint, which robot_state_publisher does not
    # publish, so the palm link is used as the RViz fixed frame.
    fixed_frame = f"{hand}_hand_palm_link"

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
            arguments=[
                "-d", str(package_share / "rviz" / "dex3.rviz"),
                "-f", fixed_frame,
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hand",
                default_value="left",
                description="Dex3-1 hand to display: left or right",
                choices=["left", "right"],
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
