from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    pkg_moveit = get_package_share_directory("abb_irb120_moveit")

    moveit_config = (
        MoveItConfigsBuilder("abb_irb120", package_name="abb_irb120_moveit")
        .robot_description(file_path="config/abb_irb120_3_60.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(pkg_moveit, "config", "moveit.rviz")

    egm_controller = Node(
        package="abb_egm_controller",
        executable="egm",
        name="egm_controller",
        parameters=[],
        output="screen",
    ),

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        rsp,
        egm_controller,
        move_group,
        rviz,
    ])