import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import cv2
import numpy as np
import os
from april_tag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.cb,
            10
        )

    def cb(self, msg):
        for det in msg.detections:
            tag_id = det.id
            pose = det.pose.pose.pose
            self.get_logger().info(
                f'Tag {tag_id}: '
                f'pos=({pose.position.x:.2f}, '
                f'{pose.position.y:.2f}, '
                f'{pose.position.z:.2f})'
            )

def main(args=None):
    rclpy.init(args=args)
    detector = AprilTagDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()