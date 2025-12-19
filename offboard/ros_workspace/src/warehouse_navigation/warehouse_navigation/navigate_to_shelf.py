# goal_pos = np.array([cx, cy]) + normal * stop_distance

# # Orientation facing the shelf (yaw)
# goal_yaw = math.atan2(-normal[1], -normal[0])

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import math
import numpy as np

SERVER_WAIT_TIMEOUT_SEC = 5.0
STOP_DISTANCE = 0.7  # meters

class ShelfGoalManager(Node):
    def __init__(self):
        super().__init__('shelf_goal_manager')

        self.shelves = []
        self.goal_active = False

        self.sub = self.create_subscription(
            Float32MultiArray,
            '/detected_shelf_info',
            self.shelf_callback,
            10
        )

        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.get_logger().info("ShelfGoalManager ready.")

        self.get_logger().info("Waiting for NavigateToPose action server...")

        if not self.action_client.wait_for_server(timeout_sec=SERVER_WAIT_TIMEOUT_SEC):
            self.get_logger().error("NavigateToPose server not available!")
            exit(1)
        self.get_logger().info("Connected to NavigateToPose server.")
    
    def shelf_callback(self, msg):

        cx, cy, nx, ny = msg.data

        # Store shelf only once
        shelf = (cx, cy, nx, ny)
        if shelf not in self.shelves:
            self.shelves.append(shelf)
            self.get_logger().info(f"Stored new shelf @ ({cx:.2f}, {cy:.2f})")

        # If no goal active → send next goal
        if not self.goal_active:
            self.send_next_goal()

    def send_next_goal(self):
        if not self.shelves:
            self.get_logger().info("No shelves left.")
            return

        cx, cy, nx, ny = self.shelves.pop(0)
        normal = np.array([nx, ny])

        # GOAL LOGIC (unchanged)
        goal_pos = np.array([cx, cy]) + normal * STOP_DISTANCE
        goal_yaw = math.atan2(-normal[1], -normal[0])

        self.goal_active = True
        self.send_goal(goal_pos[0], goal_pos[1], goal_yaw)

    def _create_quaternion_from_yaw(self, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = sy
        q.w = cy
        return q

    def send_goal(self, goal_x, goal_y, goal_yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation = self._create_quaternion_from_yaw(goal_yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return
        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully!")
        else:
            self.get_logger().warn(f"Goal failed with status: {status}")
        
        # continue to next goal
        self.goal_active = False
        self.send_next_goal()

def main():
    rclpy.init()
    node = ShelfGoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()