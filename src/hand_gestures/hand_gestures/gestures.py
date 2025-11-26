#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

from mediapipe.framework.formats.landmark_pb2 import NormalizedLandmark
from mediapipe import solutions
import numpy as np

#
# Detects hand landmarks and peforms gesture identification
#
class Gestures(Node):
    def __init__(self):
        super().__init__('gestures')
        
        self.subscriber = self.create_subscription(Image, '/webcam_raw', self.image_callback, 1)
        
        # Publisher for gesture visualization
        self.frame_publisher = self.create_publisher(Image, '/gesture_frames', 10)
        
        # Publisher for detected gesture
        self.gesture_publisher = self.create_publisher(Int32, '/gesture', 10)
        self.bridge = CvBridge()
                        
        self.model = self.build_model()
        
        # [Thumb, Pointer, Middle, Ring, Pinky]
        # (mcp, pip, tip, refrence, threshold)
        self.finger_props = [
            (2, 2, 4, 17, 0.2), 
            (5, 6, 8, 0, 0.3), 
            (9, 10, 12, 0, 0.3), 
            (13, 14, 16, 0, 0.2), 
            (17, 18, 20, 0, 0.3)
        ]
        
        # Sign language finger gestures table
        self.finger_gesture_table = [
            [0, 0, 0, 0, 0], # undefined
            [0, 1, 0, 0, 0], # 1
            [0, 1, 1, 0, 0], # 2
            [1, 1, 1, 0, 0], # 3
            [0, 1, 1, 1, 1], # 4
            [1, 1, 1, 1, 1], # 5
            [0, 1, 1, 1, 0], # 6
            [0, 1, 1, 0, 1], # 7
            [0, 1, 0, 1, 1], # 8
            [0, 0, 1, 1, 1], # 9
            [1, 0, 0, 0, 0]  # 10
        ]

    # Process message from the image topic and publish gesture data
    def image_callback(self, msg):
        if not isinstance(msg, Image):
            self.get_logger().warn("Message is not type \'Image\'")
            return

        # Peform hand landmark detection 
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
        int_msg.data = int(gesture)
        
        self.get_logger().info(int_msg)

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
    
    # Convert bgr frame to a rgb frame and runs the model on this frame
    def detect_landmarks(self, frame):
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = self.model.process(rgb_frame)
        
        return result
    
    # Draw landmarks onto a frame
    def draw_landmarks(self, frame, result):
        multi_hand_landmarks = result.multi_hand_landmarks
        frame_marked = np.copy(frame)
        
        if multi_hand_landmarks:
            for hand_landmarks in multi_hand_landmarks:
                solutions.drawing_utils.draw_landmarks(frame_marked, hand_landmarks, solutions.hands.HAND_CONNECTIONS)
                solutions.drawing_utils.draw_landmarks(frame_marked, hand_landmarks, solutions.hands.HAND_CONNECTIONS)
        
        return frame_marked

    # 
    def detect_gesture(self, landmark_results):
        multi_hand_landmarks = landmark_results.multi_hand_landmarks
        
        if multi_hand_landmarks is None:
            return -1
        
        gesture = 0
        
        for hand_landmarks in multi_hand_landmarks:
            extended = []
                            
            for points in self.finger_props:
                extended.append(self.finger_extended(hand_landmarks.landmark[points[0]], hand_landmarks.landmark[points[1]], hand_landmarks.landmark[points[2]], hand_landmarks.landmark[points[3]], points[4]))
            
            gestures_matched = np.array(self.finger_gesture_table) == np.array(extended)
            gesture = np.argmax(np.all(gestures_matched, axis=1))
        
        return gesture

    # Determine if a finger is extended
    def finger_extended(self, mcp : NormalizedLandmark, pip : NormalizedLandmark, tip : NormalizedLandmark, refrence : NormalizedLandmark, threshold : float) -> int:
        tip_pos = np.array([tip.x, tip.y])
        pip_pos = np.array([pip.x, pip.y])
        
        refrence_pos = np.array([refrence.x, refrence.y])
        mcp_pos = np.array([mcp.x, mcp.y])
        
        tip_distance = np.linalg.norm(tip_pos - refrence_pos)
        pip_distance = np.linalg.norm(pip_pos - refrence_pos)
        mcp_distance = np.linalg.norm(mcp_pos - refrence_pos)
                
        distance = tip_distance - pip_distance
                
        return int((distance / mcp_distance) > threshold)

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