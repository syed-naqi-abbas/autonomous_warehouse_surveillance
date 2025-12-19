#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

class VerticalMechanism(Node):
    def __init__(self):
        super().__init__('vertical_mechanism')

        # Create publisher for /motor_command topic
        self.stepper_publisher = self.create_publisher(String, '/motor_command', 10)
        self.qr_data_subscriber = self.create_subscription(String, '/qr_data', self.qr_data_callback, 10)

        # Publish command once after node starts
        self.timer = self.create_timer(0.1, self.main_loop)
        self.published = False

        self.steps_one_level = 1900
        self.qr_data_done = set()
        self.current_shelf_id = 1
        self.current_rack_id = 1

    def parse_qr_string(self, qr_str):
        pattern = r'R(\d+)_S(\d+)_ITM(\d+)'  # pattern to match R#, S#, ITM#
        match = re.match(pattern, qr_str)
        if match:
            rack_id, shelf_id, item_no = map(int, match.groups())
            return rack_id, shelf_id, item_no
        else:
            raise ValueError("String format does not match expected pattern")
    
    def qr_data_callback(self, msg):
        qr_string = msg.data

        rack_id, shelf_id, item_code = self.parse_qr_string(qr_string)
        
        if rack_id is None:
            return

        if(rack_id, shelf_id, item_code) in self.qr_data_done:
            return

        self.get_logger().info(f"Received QR data: {qr_string}")
        
        self.qr_data_done.add((rack_id, shelf_id, item_code))
        self.current_shelf_id = shelf_id

    def one_level_up(self):
        msg = String()
        msg.data = f'p {self.steps_one_level}'  # The motor command
        self.stepper_publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

    def come_down(self):
        msg = String()
        msg.data = f'n {self.steps_one_level*4}'  # The motor command
        self.stepper_publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

    def publish_command(self):
        if not self.published:
            
            self.published = True
            # Optional: shutdown after publishing once
            rclpy.shutdown()
    
    def main_loop(self):
        exists = any(r == self.current_rack_id and s == self.current_shelf_id for r, s, item in self.qr_data_done)
        if exists:
            if(self.current_rack_id == 5):
                self.current_shelf_id = None
                self.current_rack_id = 1
                self.come_down()
                return
            else:
                self.current_rack_id += 1
                self.one_level_up()

def main(args=None):
    rclpy.init(args=args)
    node = VerticalMechanism()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
