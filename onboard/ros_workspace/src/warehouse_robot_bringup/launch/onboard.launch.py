#!/home/comical/autonomous_warehouse_surveillance/onboard/ros_workspace/ros_env/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. Package paths
    # -------------------------------------------------------------------------
    my_robot_description_pkg = get_package_share_directory('warehouse_robot_description')
    my_robot_bringup_pkg = get_package_share_directory('warehouse_robot_bringup')
    sensors_pkg = get_package_share_directory('sensors')
    scanner_pkg = get_package_share_directory('warehouse_scanning')

    urdf_path = os.path.join(my_robot_description_pkg, 'urdf', 'my_robot.urdf.xacro')
    controller_config_path = os.path.join(my_robot_bringup_pkg, 'config', 'my_robot_controllers.yaml')

    # -------------------------------------------------------------------------
    # 2. Robot description
    # -------------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # -------------------------------------------------------------------------
    # 3. Robot State Publisher (Run ONBOARD)
    # -------------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False},
        ]
    )

    # -------------------------------------------------------------------------
    # 5. Static TFs (Optional: better to have in URDF)
    # -------------------------------------------------------------------------
    # static_lidar_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='gpu_lidar_tf_publisher',
    #     arguments=[
    #         '0.27', '0', '0', '0', '0', '0',  # 1.5708 radians = 90 degrees yaw
    #         'base_link', 'my_robot/base_link/gpu_lidar'
    #     ],
    #     parameters=[{'use_sim_time': False}],
    #     output='screen'
    # )

    # -------------------------------------------------------------------------
    # 6. ros2_control
    # -------------------------------------------------------------------------
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_config_path,
        ],
        output='both'
    )

    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager_node]
    )

    joint_state_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output='screen'
    )

    delayed_joint_state_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broad_spawner]
        )
    )

    mecanum_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller"],
        output='screen'
    )

    delayed_mecanum_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[mecanum_drive_spawner]
        )
    )

    # -------------------------------------------------------------------------
    # 7. Twist stamper (Bridge cmd_vel)
    # -------------------------------------------------------------------------
    relay_cmd_vel_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/mecanum_drive_controller/reference')
        ],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 8. Sensors
    # -------------------------------------------------------------------------
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_pkg, 'launch', 'sensors.launch.py')
        )
    )

    stepper_motor_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('stepper_motor'),
                         'launch', 'stepper_motor.launch.py')
        )
    )

    scanning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scanner_pkg, 'launch', 'qr_scanning.launch.py')
        )
    )

    rosbridge_websocket_launch = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'use_sim_time': False,
            'delay_between_messages': 0.0
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        # static_lidar_tf,
        delayed_controller_manager,
        delayed_joint_state_broad_spawner,
        delayed_mecanum_drive_spawner,
        relay_cmd_vel_node,
        sensors_launch,
        stepper_motor_node,
        # rosbridge_websocket_launch,
        # scanning_launch
    ])
    