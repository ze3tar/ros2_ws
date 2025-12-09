from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():
    # Use the xacro we just fixed in rm65_moveit_config
    robot_description_file = LaunchConfiguration("robot_description_file")

    declare_robot_description_file = DeclareLaunchArgument(
        "robot_description_file",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("rm65_moveit_config"),
                "config",
                "rm65.urdf.xacro",
            ]
        ),
        description="Absolute path to the RM65 xacro file",
    )

    # IMPORTANT: note the space after 'xacro ' so the command becomes: 'xacro /path/to/file'
    robot_description = {
        "robot_description": Command(
            ["xacro ", robot_description_file]
        )
    }

    controllers_yaml = PathJoinSubstitution(
        [
            get_package_share_directory("rm65_control"),
            "ros2_controllers.yaml",
        ]
    )

    return LaunchDescription(
        [
            declare_robot_description_file,
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_yaml],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["rm65_controller"],
                output="screen",
            ),
        ]
    )
