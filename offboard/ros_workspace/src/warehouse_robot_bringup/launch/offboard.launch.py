import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    my_robot_navigation_pkg = get_package_share_path('warehouse_navigation')
    nav2_bringup_pkg = get_package_share_path('nav2_bringup')
    ekf_config_path = os.path.join(my_robot_navigation_pkg, 'config', 'ekf.yaml')
    rviz_config_path = os.path.join(my_robot_navigation_pkg, 'config', 'default.rviz')
    nav2_params_path = os.path.join(my_robot_navigation_pkg, 'config', 'nav2_params.yaml')
    # -------------------------------------------------------------------------
    # 1. RViz2
    # -------------------------------------------------------------------------
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 2. SLAM Toolbox (Mapper)
    #    Offboard is good for this as it is heavy
    # -------------------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_link',
            'odom_frame': 'odom', 
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'transform_timeout': 0.5,
            'map_update_interval': 0.5,
            'minimum_time_interval': 0.5,
            'transform_publish_period': 0.1,
            'throttle_scans': 1,
            'max_laser_range': 12.0,
            'minimum_travel_heading': 0.1,
            'start_with_default_pose': True,
            'first_map_only': False,
            'minimum_travel_distance': 0.5,
            'distance_variance_penalty': 3.5,
            'angle_variance_penalty': 2.0,
        }],
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': False}  # False for real robot
        ],
        remappings=[
            ('odometry/filtered', 'odom')  # ← ADD THIS!
        ]
    )
    # -------------------------------------------------------------------------
    # 3. Nav2 (Navigation Stack)
    # -------------------------------------------------------------------------
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_path,
            'autostart': 'true',
        }.items()
    )

    joy_node_1 = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    joy_node_2 = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            # Safety: must hold RB to move
            'require_enable_button': True,
            'enable_button': 5,          # RB (adjust if different)
            'enable_turbo_button': 4,    # LB = turbo

            # Deadzone so robot doesn't creep when stick is slightly off-center
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,     # Hz, smooth updates while holding

            # --- Holonomic linear controls (x, y) ---
            # Left stick vertical (axes[1]) -> forward/back
            # Left stick horizontal (axes[0]) -> strafe left/right
            'axis_linear': {
                'x': 1,   # forward/back (left stick up/down)
                'y': 0    # strafe (left stick left/right)
            },

            # Normal speed
            'scale_linear': {
                'x': 0.4,   # m/s
                'y': 0.4
            },

            # Turbo speed (while LB + RB held)
            'scale_linear_turbo': {
                'x': 0.8,
                'y': 0.8
            },

            # --- Angular control (yaw) ---
            # Right stick horizontal (axes[3]) -> rotate
            'axis_angular': {
                'yaw': 3
            },

            'scale_angular': {
                'yaw': 1.0   # rad/s normal
            },
            'scale_angular_turbo': {
                'yaw': 2.0   # rad/s turbo
            },
        }]
    )

    motor_command_node = Node(
        package='warehouse_robot_bringup',
        executable='motor_command_controller.py',
        name='motor_command_controller',
        output='screen'
    )

    laser_filter_node = Node(
        package='warehouse_robot_bringup',
        executable='laser_filter.py',
        name='laser_filter',
        output='screen'
    )

    # velocity_scaler_node = Node(
    #     package='warehouse_robot_bringup',
    #     executable='velocity_scaler.py',
    #     name='velocity_scaler',
    #     output='screen'
    # )

    return LaunchDescription([
        rviz2_node,
        slam_node,
        nav2_bringup_launch,
        robot_localization_node,
        joy_node_1,
        joy_node_2,
        motor_command_node,
        laser_filter_node,
        # velocity_scaler_node,
    ])

                                                                                                                                                                                                                                                                                                                           