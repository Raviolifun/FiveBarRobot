import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory("five_bar_robot")


def generate_launch_description():
    ld = LaunchDescription()

    pose_demo = Node(
        package="five_bar_robot",
        executable="pose_demo",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        namespace="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", config_dir + "/rviz/robot_pose.rviz"],
    )

    ld.add_action(pose_demo)
    ld.add_action(rviz)
    return ld
