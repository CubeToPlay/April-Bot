#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanInfFixer(Node):
    def __init__(self):
        super().__init__('scan_inf_fixer')
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info("Scan Inf Fixer started, republishing to /scan_fixed")

    def scan_callback(self, msg: LaserScan):
        # Create a new LaserScan to publish
        fixed_scan = LaserScan()
        fixed_scan.header = msg.header
        fixed_scan.angle_min = msg.angle_min
        fixed_scan.angle_max = msg.angle_max
        fixed_scan.angle_increment = msg.angle_increment
        fixed_scan.time_increment = msg.time_increment
        fixed_scan.scan_time = msg.scan_time
        fixed_scan.range_min = msg.range_min
        fixed_scan.range_max = msg.range_max

        # Convert ranges: replace 0 or NaN with max range if needed
        fixed_scan.ranges = []
        for r in msg.ranges:
            if r == 0.0 or math.isnan(r):
                fixed_scan.ranges.append(float('inf'))
            else:
                fixed_scan.ranges.append(r)

        # Intensities: make a list of floats
        fixed_scan.intensities = list(msg.intensities)

        self.pub.publish(fixed_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanInfFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
