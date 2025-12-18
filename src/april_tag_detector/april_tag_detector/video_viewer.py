import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import yaml
import os

class VideoView(Node):
    """Video viewer to see through the robot's camera and to see if the robot sees an AprilTag"""
    
    def __init__(self):
        super().__init__('video_viewer')

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("apriltag_topic", "/apriltag_detections_image")
        img_topic = self.get_parameter("image_topic").value
        apriltag_topic = self.get_parameter("apriltag_topic").value

        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Current frame
        self.current_frame = None
        self.current_apriltag_frame = None
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            img_topic,
            self.image_callback,
            10
        )

        self.april_sub = self.create_subscription(
            Image,
            apriltag_topic,
            self.apriltag_callback,
            10
        )
        
        # Create window and trackbars
        self.window_name = 'Video View'
        cv2.namedWindow(self.window_name)
    
    def image_callback(self, msg):
        """Process incoming images."""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
    
    def process_and_display(self):
        """Process current frame with current HSV settings and display."""
        if self.current_frame is None and self.current_apriltag_frame is None:
            return
        if self.current_frame is None:
            display = self.current_apriltag_frame
        elif self.current_apriltag_frame is None:
            display = self.current_frame
        else:
            display = self.current_apriltag_frame
        
        cv2.imshow(self.window_name, display)

    def apriltag_callback(self, msg):
        """Process incoming images."""
        try:
            self.current_apriltag_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')


    def run(self):
        """Main loop for processing and display."""
        while rclpy.ok():
            # Process ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # Process and display current frame
            self.process_and_display()
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quitting...')
                break
        
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    tuner = VideoView()
    
    try:
        tuner.run()
    except KeyboardInterrupt:
        pass
    finally:
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
