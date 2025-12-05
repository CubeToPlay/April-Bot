#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

# 
# Publishes webcam stream to the topic /webcam_raw
#  
class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        self.publisher = self.create_publisher(Image, '/webcam_raw', 10)
    
        # Calls image callback every set time
        self.timer = self.create_timer(0.05, self.image_callback)
        
        self.bridge = CvBridge()
        self.capture = cv2.VideoCapture(0)
        
        # https://forum.opencv.org/t/videoio-v4l2-dev-video0-select-timeout/8822/5
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    
    # Publishes an image from the webcam
    def image_callback(self):        
        if not self.capture.isOpened():
            self.get_logger().error("Unable to open webcam for video capture")
            self.cleanup()
        
        success, frame = self.capture.read()
        
        if not success:
            self.get_logger().warn("Unable to read video frame")
            return
        
        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        self.publisher.publish(image_msg)

    # Clean the node for destruction
    def cleanup(self):
        self.get_logger().info("Webcam Publisher Node Shutting Down")
        self.capture.release()
        self.destroy_node()

#
# Spin up node 
# On keyboard interrupt, just clean up
#
def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    
    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass
    
    webcam_publisher.cleanup()
    
    if rclpy.ok():
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()