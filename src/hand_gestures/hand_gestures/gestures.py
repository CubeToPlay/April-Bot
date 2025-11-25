#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

from mediapipe import solutions
import numpy as np

#
# Detects hand landmarks and peforms gesture identification
#
class Gestures(Node):
    def __init__(self):
        super().__init__('gestures')
        
        self.subscriber = self.create_subscription(Image, '/webcam_raw', self.image_callback, 1)
        self.frame_publisher = self.create_publisher(Image, '/gesture_frames', 10)
        self.gesture_publisher = self.create_publisher(Int32, '/gesture', 10)
        self.bridge = CvBridge()
                        
        self.model = self.build_model()

    
    def image_callback(self, msg):
        if not isinstance(msg, Image):
            self.get_logger().warn("Message is not type \'Image\'")
            return

        # Peform landmark detection 
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        frame = cv2.flip(frame, 1)
        landmark_results = self.detect_landmarks(frame)
            
        # Publish hand tracking visualization
        image_marked = self.draw_landmarks(frame, landmark_results)
        image_msg = self.bridge.cv2_to_imgmsg(image_marked, encoding='bgr8')
        self.frame_publisher.publish(image_msg)
        
        # Publish gesture identification
        gesture = self.detect_gesture(landmark_results)
                
        int_msg = Int32()
        int_msg.data = gesture
        
        # self.get_logger().info(int_msg)

        print(gesture)
        
        self.gesture_publisher.publish(int_msg)
    
    # https://github.com/patience60-svg/gesture-control-ros2
    # https://mediapipe.readthedocs.io/en/latest/solutions/hands.html
    def build_model(self):
        return solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
    
    def detect_landmarks(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = self.model.process(rgb_frame)
        
        return result
            
    def draw_landmarks(self, frame, result):
        multi_hand_landmarks = result.multi_hand_landmarks
        frame_marked = np.copy(frame)
        
        if multi_hand_landmarks:
            for hand_landmarks in multi_hand_landmarks:
                solutions.drawing_utils.draw_landmarks(frame_marked, hand_landmarks, solutions.hands.HAND_CONNECTIONS)
                solutions.drawing_utils.draw_landmarks(frame_marked, hand_landmarks, solutions.hands.HAND_CONNECTIONS)
        
        return frame_marked

    def detect_gesture(self, landmark_results):
        multi_hand_landmarks = landmark_results.multi_hand_landmarks
        
        if multi_hand_landmarks is None:
            return -1
        
        return 0

    # def finger_extended(self, tip, )

    def cleanup(self):
        self.get_logger().info("Gestures Node Shutting Down")

#
# Spin up node 
# On keyboard interrupt, just clean up
#
def main(args=None):
    rclpy.init(args=args)
    gestures = Gestures()
    
    try:
        rclpy.spin(gestures)
    except KeyboardInterrupt:
        return
    finally:
        gestures.cleanup()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()