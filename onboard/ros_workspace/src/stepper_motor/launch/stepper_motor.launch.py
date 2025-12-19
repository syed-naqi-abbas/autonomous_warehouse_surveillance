from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('stepper_motor'),
        'config'
    )
    config_file = os.path.join(config_dir, 'stepper_motor_params.yaml')
    
    return LaunchDescription([
        Node(
            package='stepper_motor',
            executable='stepper_motor_node',
            name='stepper_motor_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='stepper_motor',
            executable='stepper_motor_teleop',
            name='stepper_motor_teleop',
            output='screen'
        )
    ])