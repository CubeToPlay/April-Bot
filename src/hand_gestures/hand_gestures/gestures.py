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

import os

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
        
        self.landmarker = self.build_landmarker()
                
    def image_callback(self, msg):
        if not isinstance(msg, Image):
            self.get_logger().warn("Message is not type \'Image\'")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame = cv2.flip(frame, 1)
        
        self.detect_landmarks(frame)
        
    def build_landmarker(self):
        # Get path of the hand landmarker model file
        _, package_path = get_resource("packages", 'hand_gestures')
        model_path = os.path.join(package_path, "share", 'hand_gestures', "resource", "hand_landmarker.task")
        
        # Set options for the hand landmarker model
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options = mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode = mp.tasks.vision.RunningMode.IMAGE,
            num_hands = 2,
            min_hand_detection_confidence = 0.1, # lower than value to get predictions more often
            min_hand_presence_confidence = 0.1, # lower than value to get predictions more often
            min_tracking_confidence = 0.1, # lower than value to get predictions more often   
        )
        
        return mp.tasks.vision.HandLandmarker.create_from_options(options)
    
    def detect_landmarks(self, frame):
        cv2.imshow('restul', frame)
        rgb_frame = mp.Image(data=frame, image_format=mp.ImageFormat.SRGB)
        result = self.landmarker.detect(rgb_frame)
        # cv2.imshow('restul', frame)
        
        image_marked = self.draw_landmarks(frame, result)
        # cv2.imshow('restul', cv2.cvtColor(image_marked, cv2.COLOR_RGB2BGR))
        cv2.imshow('restul', image_marked)
        if cv2.waitKey(1):
            pass
            
    def draw_landmarks(self, rgb_frame, result):
        hand_landmarks_list = result.hand_landmarks
        handedness_list = result.handedness
        annotated_image = np.copy(rgb_frame)

        # Loop through the detected hands to visualize.
        for idx in range(len(hand_landmarks_list)):
            hand_landmarks = hand_landmarks_list[idx]
            handedness = handedness_list[idx]

            # Draw the hand landmarks.
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            
            hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
            ])
            
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                hand_landmarks_proto,
                solutions.hands.HAND_CONNECTIONS,
                solutions.drawing_styles.get_default_hand_landmarks_style(),
                solutions.drawing_styles.get_default_hand_connections_style()
            )

            # Get the top left corner of the detected hand's bounding box.
            height, width, _ = annotated_image.shape
            x_coordinates = [landmark.x for landmark in hand_landmarks]
            y_coordinates = [landmark.y for landmark in hand_landmarks]
            text_x = int(min(x_coordinates) * width)
            text_y = int(min(y_coordinates) * height) - MARGIN

            # Draw handedness (left or right hand) on the image.
            cv2.putText(annotated_image, f"{handedness[0].category_name}",
                        (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                        FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)

        return annotated_image             
    
    def cleanup(self):
        self.landmarker.close()
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