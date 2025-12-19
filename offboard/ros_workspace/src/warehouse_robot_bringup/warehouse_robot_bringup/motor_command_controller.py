#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class MotorCommandController(Node):
    def __init__(self):
        super().__init__('motor_command_controller')
        
        # Subscribe to joy input with higher queue size
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            20
        )
        
        # Publish motor commands
        self.motor_pub = self.create_publisher(
            String,
            '/motor_command',
            20
        )
        
        # Track current button states
        self.current_buttons = []
        self.active_command = None
        
        # Timer for continuous publishing at 50 Hz (20ms interval)
        self.publish_timer = self.create_timer(0.02, self.publish_command)
        
        self.get_logger().info("Motor Command Controller started (50 Hz)")
        self.get_logger().info("X button (2) -> 'p 20' (positive rotation - continuous)")
        self.get_logger().info("A button (0) -> 'n 20' (negative rotation - continuous)")
    
    def joy_callback(self, msg: Joy):
        """Store current button states"""
        self.current_buttons = msg.buttons
    
    def publish_command(self):
        """Publish command at 50 Hz for smooth continuous motion"""
        if not self.current_buttons:
            return
        
        buttons = self.current_buttons
        
        # X button (index 2) -> 'p 20' (positive rotation)
        if buttons[2] == 1:
            if self.active_command != 'p 20':
                self.get_logger().info("X pressed -> Sending 'p 20' (continuous 50 Hz)")
                self.active_command = 'p 20'
            cmd = String(data='p 20')
            self.motor_pub.publish(cmd)
        
        # A button (index 0) -> 'n 20' (negative rotation)
        elif buttons[0] == 1:
            if self.active_command != 'n 20':
                self.get_logger().info("A pressed -> Sending 'n 20' (continuous 50 Hz)")
                self.active_command = 'n 20'
            cmd = String(data='n 20')
            self.motor_pub.publish(cmd)
        
        # Button released - send stop command immediately
        else:
            if self.active_command is not None:
                self.get_logger().info("Button released -> Sending STOP command")
                self.active_command = None
                # Send explicit stop command
                stop_cmd = String(data='stop')
                self.motor_pub.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()