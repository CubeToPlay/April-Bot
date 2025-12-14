import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanInfFixer(Node):
    def __init__(self):
        super().__init__('scan_inf_fixer')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_fixed',
            10
        )

        self.get_logger().info("Scan Inf Fixer started, republishing to /scan_fixed")

    def scan_callback(self, msg: LaserScan):
        fixed_scan = LaserScan()
        fixed_scan.header = msg.header
        fixed_scan.angle_min = msg.angle_min
        fixed_scan.angle_max = msg.angle_max
        fixed_scan.angle_increment = msg.angle_increment
        fixed_scan.time_increment = msg.time_increment
        fixed_scan.scan_time = msg.scan_time
        fixed_scan.range_min = msg.range_min
        fixed_scan.range_max = msg.range_max
        fixed_scan.intensities = msg.intensities.copy()

        # Replace inf with range_max
        fixed_scan.ranges = [
            msg.range_max if math.isinf(r) else r
            for r in msg.ranges
        ]

        self.publisher.publish(fixed_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanInfFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
