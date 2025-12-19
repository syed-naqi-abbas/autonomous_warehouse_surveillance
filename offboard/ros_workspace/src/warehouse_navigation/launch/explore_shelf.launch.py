from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    frontier = Node(
        package='warehouse_navigation',
        executable='frontier_exploration.py',
        name='frontier_exploration'
    )

    shelf_detector = Node(
        package='warehouse_navigation',
        executable='shelf_detector.py',
        name='shelf_detector'
    )

    navigator = Node(
        package='warehouse_navigation',
        executable='navigate_to_shelf.py',
        name='shelf_navigator'
    )

    return LaunchDescription([
        frontier,

        # When frontier finishes, start both shelf_detector and navigator simultaneously
        RegisterEventHandler(
            OnProcessExit(
                target_action=frontier,
                on_exit=[shelf_detector, navigator]
            )
        )
    ])