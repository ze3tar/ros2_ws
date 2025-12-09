from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_pkg = get_package_share_directory('rm65_description')
    urdf_path = os.path.join(description_pkg, 'urdf', 'rm_65_6f_description.urdf')

    # robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # joint_state_publisher_gui
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    return LaunchDescription([rsp, jsp])

