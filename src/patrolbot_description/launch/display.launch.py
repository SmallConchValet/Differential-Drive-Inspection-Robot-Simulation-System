from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
def generate_launch_description():
 u=str(Path(get_package_share_directory('patrolbot_description'))/'urdf/patrolbot.urdf'); robot=Path(u).read_text()
 return LaunchDescription([Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui'),Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':robot}]),Node(package='rviz2',executable='rviz2')])
