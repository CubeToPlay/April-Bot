#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import mediapipe as mp

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np

from ament_index_python.resources import get_resource

MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green

class Gestures(Node):
    def __init__(self):
        super().__init__('gestures')
        
        self.subscriber = self.create_subscription(Image, '/webcam_raw', self.image_callback, 1)
        self.bridge = CvBridge()
                        
        self.model = self.build_model()
                
    def image_callback(self, msg):
        if not isinstance(msg, Image):
            self.get_logger().warn("Message is not type \'Image\'")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame = cv2.flip(frame, 1)
        
        landmark_result = self.detect_landmarks(frame)
        
        print(landmark_result)
        
        image_marked = self.draw_landmarks(frame, landmark_result)
        
        cv2.imshow('Gesture', image_marked)
        if cv2.waitKey(1):
            pass
    
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
            
    def draw_landmarks(self, rgb_frame, result):
        multi_hand_landmarks = result.multi_hand_landmarks
        frame_marked = np.copy(rgb_frame)
        
        if multi_hand_landmarks:
            for hand_landmarks in multi_hand_landmarks:
                solutions.drawing_utils.draw_landmarks(frame_marked, hand_landmarks, solutions.hands.HAND_CONNECTIONS)
        
        return frame_marked

    def cleanup(self):
        self.get_logger().info("Gestures Node Shutting Down")

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