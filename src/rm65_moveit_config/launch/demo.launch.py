from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Load MoveIt configs
    moveit_config = MoveItConfigsBuilder(
        "rm65", package_name="rm65_moveit_config"
    ).to_moveit_configs()

    # Path to MoveGroup launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_config.package_path,
                "launch",
                "move_group.launch.py"
            )
        )
    )

    # Path to RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_config.package_path,
                "launch",
                "moveit_rviz.launch.py"
            )
        )
    )

    # Path to ros2_control launch
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_config.package_path,
                "launch",
                "rsp.launch.py"
            )
        )
    )

    # Robot State Publisher (URDF → TF)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="both",
    )

    return LaunchDescription([
        robot_state_pub,
        ros2_control_launch,
        move_group_launch,
        rviz_launch,
    ])
