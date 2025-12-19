from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_scanning',
            executable='qr_detection',
            name='qr_viewer',
            output='screen',
            parameters=[],
        ),
    ])
