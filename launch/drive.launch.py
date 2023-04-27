import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    odrive_driver = Node(
        package="five_bar_robot",
        executable="driver",
        output="screen",
        parameters=[
            {
                "theta_offset1": 160.0,
                "theta_offset2": 20.0,
            }
        ],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[{"sticky_buttons": True}],
    )

    # camera_launch1 = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "launch",
    #         "flir_spinnaker_ros2",
    #         "blackfly_s.launch.py",
    #         "camera_name:=blackfly_0",
    #         "serial:=" "18484101" "",
    #     ],
    # )

    # camera_launch2 = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "launch",
    #         "flir_spinnaker_ros2",
    #         "blackfly_s.launch.py",
    #         "camera_name:=blackfly_0",
    #         "serial:=" "18382965" "",
    #     ],
    # )

    ld.add_action(odrive_driver)
    ld.add_action(joy_node)
    # ld.add_action(camera_launch1)
    # ld.add_action(camera_launch2)

    return ld


# ros2 launch flir_spinnaker_ros2 blackfly_s.launch.py camera_name:=blackfly_0 serial:="'20435008'"
