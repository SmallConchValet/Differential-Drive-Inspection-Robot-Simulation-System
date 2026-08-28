from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    package_dir = get_package_share_directory(
        'patrolbot_description'
    )

    urdf_file = os.path.join(
        package_dir,
        'urdf',
        'patrolbot.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description':robot_description}
            ]
        ),

        Node(
            package=='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),

        Node(
            package='rviz2',
            executable='rviz2'
        )
    
    ])