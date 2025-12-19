from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the other launch file
    lidar_launch_path = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )

    return LaunchDescription([
        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     name='logitech_camera',
        #     output='screen',
        #     parameters=[
        #         {'video_device': '/dev/video0'}
        #     ]
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),
        Node(
            package='sensors',
            executable='bno055_node',
            name='bno055',
            output='screen',
            parameters=[  # Optional: pass parameters if needed
                {'i2c_port': '/dev/i2c-1'}  # example, adjust if needed
            ]
        )
    ])
