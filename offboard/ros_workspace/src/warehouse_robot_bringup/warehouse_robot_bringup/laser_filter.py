#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LidarRightFilter(Node):
    def __init__(self):
        super().__init__('laser_filter')

        # Create a "Best Effort" QoS profile to match standard LiDAR drivers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan_unfiltered',
            self.scan_callback,
            qos
        )
        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        # ------------------- ANGLE CONFIGURATION -------------------
        # Goal: Keep Right Side (0 to -180) PLUS 24 deg extensions.
        # This implies we want to KEEP:
        #   1. The Right Side: 0.0 to -3.14 (negative angles)
        #   2. The Front Extension: 0.0 to +0.42 rad (+24 deg)
        #   3. The Back Extension: -3.14 wraps to +3.14. 
        #      We want 24 deg past the back, which is +3.14 down to +2.72 rad (+156 deg)
        #
        # Therefore, the only data we want to DISCARD is the "Left Wedge":
        #   From +24 degrees to +156 degrees.

        buffer_angle = math.radians(20) 
        
        self.ignore_start = buffer_angle            # +24 degrees
        self.ignore_end = math.pi - buffer_angle    # 180 - 24 = +156 degrees

        self.get_logger().info(
            f'LiDAR Filter Started. Ignoring Left Wedge: {math.degrees(self.ignore_start):.1f}° to {math.degrees(self.ignore_end):.1f}°'
        )
        # -----------------------------------------------------------

    def scan_callback(self, scan: LaserScan):
        new_scan = LaserScan()

        # Copy metadata exactly from the original scan
        new_scan.header = scan.header
        new_scan.angle_min = scan.angle_min
        new_scan.angle_max = scan.angle_max
        new_scan.angle_increment = scan.angle_increment
        new_scan.time_increment = scan.time_increment
        new_scan.scan_time = scan.scan_time
        new_scan.range_min = scan.range_min
        new_scan.range_max = scan.range_max
        
        new_scan.ranges = []
        new_scan.intensities = []

        # Start tracking the angle from the minimum angle of the scan
        current_angle = scan.angle_min

        for i in range(len(scan.ranges)):
            
            # LOGIC:
            # If the angle is INSIDE the "Ignore Zone" (Left side), mask it.
            # Otherwise (Right side, Front Ext, Back Ext), keep it.
            
            # We strictly check if we are in the positive "Left Wedge"
            is_in_ignore_zone = (current_angle > self.ignore_start) and \
                                (current_angle < self.ignore_end)

            if is_in_ignore_zone:
                # Mask out data (set to Infinity)
                new_scan.ranges.append(float('inf')) 
                if scan.intensities:
                    new_scan.intensities.append(0.0)
            else:
                # Keep valid data
                new_scan.ranges.append(scan.ranges[i])
                if scan.intensities:
                    new_scan.intensities.append(scan.intensities[i])

            # Move to the next angle
            current_angle += scan.angle_increment

        # Publish the filtered scan
        self.pub.publish(new_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRightFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()