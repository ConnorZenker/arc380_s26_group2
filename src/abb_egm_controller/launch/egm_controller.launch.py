from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("udp_port", default_value="6511"),
            DeclareLaunchArgument("unix_offset", default_value="0"),
            DeclareLaunchArgument("send_latency", default_value="0"),
            Node(
                package="abb_egm_controller",
                executable="egm",
                name="egm_controller",
                parameters=[
                    {"udp_port": LaunchConfiguration("udp_port"), "unix_offset": LaunchConfiguration("unix_offset"), "send_latency": LaunchConfiguration("send_latency")}
                ],
                output="screen",
            ),
        ]
    )
