#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityScaler(Node):
    def __init__(self):
        super().__init__('velocity_scaler')

        # Declare scaling factor parameter
        self.declare_parameter('scale_factor', 10.0)
        self.scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value

        # Subscribers & publishers
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel_slow',
            self.cmd_vel_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info(f"Velocity scaler started with factor: {self.scale_factor}")

    def cmd_vel_callback(self, msg: Twist):
        scaled = Twist()

        # Apply scaling to linear and angular components
        scaled.linear.x = msg.linear.x * self.scale_factor
        scaled.linear.y = msg.linear.y * self.scale_factor
        scaled.linear.z = msg.linear.z * self.scale_factor
        
        scaled.angular.x = msg.angular.x * self.scale_factor
        scaled.angular.y = msg.angular.y * self.scale_factor
        scaled.angular.z = msg.angular.z * self.scale_factor

        self.pub.publish(scaled)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
