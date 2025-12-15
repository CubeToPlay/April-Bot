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
        super().__init__('detector')
        
        self.declare_parameter('tag_size', 0.5) # meters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('tag_dir', '')

        self.tag_size = self.get_parameter('tag_size').value
        """Gives physical size of the AprilTags in meters"""
        self.camera_topic = self.get_parameter('camera_topic').value
        """The publisher where it receives images from"""
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        """The publisher where it receives cmaera calibration from"""
        self.publish_tf_flag = self.get_parameter('publish_tf').value
        """The flag to determine if it should publish TF transforms"""
        self.tag_dir = self.get_parameter('tag_dir').value
        """The directory that containsthe tag images"""

        self.max_hamming = 8

        self.min_confirm_time = 0.2  # seconds
        self.max_center_jump = 50.0  # pixels

        self.bridge = CvBridge()

        self.tf_broadcaster = TransformBroadcaster(self)
        """Broadcasts tag poses as TF transforms"""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = 'camera_link'

        self.tags = {}
        """Dictionary that stores the loded tag images"""

        if self.tag_dir:
            self.load_tags()
        self.valid_tag_codes = self.load_tag36h11_codes()
        
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        """Subscriber that receives camera frames"""
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        """Subscriber that receives calibration data"""

        # Publishers
        self.detection_pub = self.create_publisher(PoseStamped, '/apriltag_detections', 10)
        """Publishes individual tag poses"""
        self.detections_array_pub = self.create_publisher(AprilTagDetectionArray, '/april_tags', 10)
        """Publishes an array of all detected AprilTags"""
        self.visualization_pub = self.create_publisher(Image, '/apriltag_detections_image', 10)
        """Publishes visualization of the AprilTags with drawn detections"""
    
    def decode_apriltag(self, img):
        """
        Universal decoder for both templates and detected tags.
        Input: 240x240 grayscale image containing 8x8 structure (border + 6x6 data + border)
        Output: 36-bit code (6x6 flattened array)
        """
        
        # The image is 240x240, representing an 8x8 grid
        GRID = 8
        CELL = img.shape[0] // GRID  # 30 pixels per cell
        
        bits_8x8 = np.zeros((GRID, GRID), dtype=np.uint8)
        
        # Extract all 8x8 cells
        margin = 3  # Small margin to avoid edge effects
        for y in range(GRID):
            for x in range(GRID):
                y_start = y * CELL + margin
                y_end = (y + 1) * CELL - margin
                x_start = x * CELL + margin
                x_end = (x + 1) * CELL - margin
                
                cell = img[max(0, y_start):min(img.shape[0], y_end),
                        max(0, x_start):min(img.shape[1], x_end)]
                
                # Use median for robustness
                mean_val = np.median(cell)
                # Black = 1, White = 0
                bits_8x8[y, x] = 1 if mean_val < 128 else 0
        
        # Normalize polarity: corners should be BLACK (1) - part of the border
        corner_bits = [bits_8x8[0,0], bits_8x8[0,7], bits_8x8[7,0], bits_8x8[7,7]]
        if np.mean(corner_bits) < 0.5:  # If corners are mostly white
            bits_8x8 = 1 - bits_8x8
        
        # Extract inner 6x6 data region (skip the 1-cell border)
        bits_6x6 = bits_8x8[1:7, 1:7]
        ones = np.sum(bits_6x6)
        if ones < 2 or ones > 32:
            return None
        
        return bits_6x6.flatten()
            
    def load_tags(self):
        """Load AprilTag images from directory"""
        if not os.path.exists(self.tag_dir):
            self.get_logger().warn(f'Template directory not found: {self.tag_dir}')
            return
        
        for filename in os.listdir(self.tag_dir):
            if filename.startswith('tag36h11-') and filename.endswith('.png'):
                try:
                    tag_id = int(filename.split('-')[1].split('.')[0])
                except:
                    self.get_logger().warn(f'Could not parse tag ID from: {filename}')
                    continue
                
                filepath = os.path.join(self.tag_dir, filename)
                image = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
                
                if image is not None:
                    # Resize to standard size
                    image = cv2.resize(image, (240, 240), interpolation=cv2.INTER_AREA)
                    
                    # Apply clean threshold
                    _, image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
                    
                    self.tags[tag_id] = image
                    self.get_logger().info(f'Loaded tag ID {tag_id}')
        
        self.get_logger().info(f'Loaded {len(self.tags)} tag images')
    
    def decode_template(self, img):
        """Decode template - 8x8 structure (1 border + 6 data + 1 border)"""
        
        # The image is 240x240, representing an 8x8 structure
        GRID = 8  # Changed from 6 to 8
        CELL = img.shape[0] // GRID  # Should be 30 pixels per cell
        
        bits_8x8 = np.zeros((GRID, GRID), dtype=np.uint8)
                
        # Extract all 8x8 cells
        for y in range(GRID):
            for x in range(GRID):
                # Sample from center of cell to avoid edge effects
                margin = 3
                cell = img[y*CELL + margin:(y+1)*CELL - margin,
                        x*CELL + margin:(x+1)*CELL - margin]
                
                mean_val = np.mean(cell)
                # Black = 1, White = 0
                bits_8x8[y, x] = 1 if mean_val < 128 else 0
        
        # Extract inner 6x6 (skip the border)
        bits_6x6 = bits_8x8[1:7, 1:7]
        
        return bits_6x6.flatten()
    
    def draw_grid(self, img, grid=6):
        """Draw grid overlay - default to 8x8 to show full structure"""
        h, w = img.shape[:2]
        step = h // grid
        
        if len(img.shape) == 2:
            out = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            out = img.copy()
        
        # Draw grid
        for i in range(1, grid):
            # Highlight border rows/cols in RED
            if i == 1 or i == grid - 1:
                color = (0, 0, 255)  # Red for border
                thickness = 2
            else:
                color = (0, 255, 0)  # Green for data
                thickness = 1
                
            cv2.line(out, (0, i*step), (w, i*step), color, thickness)
            cv2.line(out, (i*step, 0), (i*step, h), color, thickness)
        
        return out
    
    def decode_quad(self, warped):
        """Decode a warped quad from camera"""
        
        # Check if image is properly binary
        unique_vals = np.unique(warped)
        
        # If not binary, threshold it again
        if len(unique_vals) > 10:  # Not binary
            _, warped = cv2.threshold(warped, 127, 255, cv2.THRESH_BINARY)
        
        # Use the unified decode function
        detected_code = self.decode_apriltag(warped)
        if detected_code is None:
            return None
        detected_6x6 = detected_code.reshape(6, 6)
        
        best_id = None
        best_score = -1

        # Try all 4 rotations
        for rot in range(4):
            rotated_code = np.rot90(detected_6x6, rot).flatten()

            for tag_id, template in self.valid_tag_codes.items():
                score = np.sum(rotated_code == template)
                
                if score > best_score:
                    best_score = score
                    best_id = tag_id

        threshold = 36 - self.max_hamming
                
        if best_score >= threshold:
            # Show visualization - now should show clean black/white cells
            cv2.imshow(f"Tag {best_id}", self.draw_grid(warped, grid=8))
            cv2.waitKey(1)
            return best_id
        
        return None


    def load_tag36h11_codes(self):
        """Load codes using the SAME decoder as detection"""
        codes = {}
        
        for tag_id, img in self.tags.items():
            # Use the SAME decode function as we use for detected tags
            code = self.decode_apriltag(img)
            codes[tag_id] = code
            
            self.get_logger().info(f'\nTag {tag_id} template code:\n{code.reshape(6,6)}')
        
        self.get_logger().info(f"Loaded {len(codes)} tag codes")
        return codes
    
    def camera_info_callback(self, msg):
        """Store camera calibration"""
        # This extracts camera info needed for 3D pose estimation
        '''
        Extracts the camera matrix (3x3): Focal lengths and optical center:
            [fx   0   cx]
            [0   fy   cy]
            [0    0    1]
        dist_coeffs are the lens distortion coefficients
        '''
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
            self.camera_frame = msg.header.frame_id
            self.get_logger().info(f'Camera calibration received. Frame: {self.camera_frame}')

    def order_points(self, pts):
        rect = np.zeros((4,2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    def find_apriltags_contours(self, image, warp_size=240):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        debug_image = image.copy()
        
        # Threshold for finding contours
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        _, thresh = cv2.threshold(blur, 0, 255,
                                cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        detected_tags = []

        if hierarchy is None:
            return detected_tags, image

        for i, cnt in enumerate(contours):
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.015 * peri, True)
            if len(approx) < 4:
                cv2.polylines(debug_image, [approx.astype(int)], True, (0, 0, 255), 1)
                continue

            cv2.polylines(debug_image, [approx.astype(int)], True, (255, 0, 0), 2)
            
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            quad = self.order_points(box)

            # Square-ish check
            w = np.linalg.norm(quad[0] - quad[1])
            h = np.linalg.norm(quad[0] - quad[3])
            if h > 0 and not 0.6 < w / h < 1.6:
                continue

            # Warp - IMPORTANT: Warp from ORIGINAL GRAY, not from thresh
            dst = np.array([
                [0, 0],
                [warp_size - 1, 0],
                [warp_size - 1, warp_size - 1],
                [0, warp_size - 1]
            ], dtype=np.float32)

            M = cv2.getPerspectiveTransform(quad.astype(np.float32), dst)
            
            # *** KEY FIX: Warp from original gray, not from thresh ***
            warped = cv2.warpPerspective(gray, M, (warp_size, warp_size))
            # if warped.std() < 20:
            #     continue
            # *** THEN apply a clean threshold to the warped image ***
            # Try different threshold methods to see which works best:
            
            # Option 1: Otsu's method (automatic threshold)
            _, warped_thresh = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            
            # Option 2: Simple fixed threshold
            # _, warped_thresh = cv2.threshold(warped, 127, 255, cv2.THRESH_BINARY)
            
            # Option 3: Adaptive threshold (if lighting varies)
            # warped_thresh = cv2.adaptiveThreshold(
            #     warped, 255,
            #     cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            #     cv2.THRESH_BINARY,
            #     31, 10
            # )
            
            # Use the thresholded version for decoding
            tag_id = self.decode_quad(warped_thresh)
            
            if tag_id is None:
                continue

            detected_tags.append({
                "id": tag_id,
                "corners": quad,
                "center": quad.mean(axis=0)
            })

        return detected_tags, debug_image

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        detected_tags, debug_image = self.find_apriltags_contours(cv_image)
        vis_image = debug_image
        detections_array = AprilTagDetectionArray()
        detections_array.header.stamp = self.get_clock().now().to_msg()
        detections_array.header.frame_id = self.camera_frame

        for tag in detected_tags:
            tag_id = tag['id']
            center = tag['center']
            corners = tag['corners']
            pose = self.estimate_pose_pnp(corners)
            if pose is not None:
                self.publish_detection(tag_id, pose)
                detection = AprilTagDetection()
                detection.id = int(tag_id)
                detection.pose.position.x = float(pose['position'][0])
                detection.pose.position.y = float(pose['position'][1])
                detection.pose.position.z = float(pose['position'][2])
                detection.pose.orientation.x = pose['orientation'][0]
                detection.pose.orientation.y = pose['orientation'][1]
                detection.pose.orientation.z = pose['orientation'][2]
                detection.pose.orientation.w = pose['orientation'][3]
                detection.corners = corners.flatten().tolist()
                detection.size = float(self.tag_size)
                detection.hamming = int(0)
                detection.decision_margin = float(0.8)
                detections_array.detections.append(detection)
                cv2.polylines(vis_image, [corners.astype(int)], True, (0,255,0), 2)

        if len(detections_array.detections) > 0:
            self.detections_array_pub.publish(detections_array)

        self.visualization_pub.publish(self.bridge.cv2_to_imgmsg(vis_image, "bgr8"))

    def estimate_pose_pnp(self, corners):
        if self.camera_matrix is None:
            return None

        half = self.tag_size / 2.0
        object_points = np.array([
            [-half,  half, 0],   # TL
            [ half,  half, 0],   # TR
            [ half, -half, 0],   # BR
            [-half, -half, 0],   # BL
        ], dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            corners.astype(np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success:
            return None

        R_cv, _ = cv2.Rodrigues(rvec)

        # OpenCV → ROS rotation
        R_cv_to_ros = np.array([
            [0,  0,  1],
            [-1, 0,  0],
            [0, -1,  0]
        ])
        R_ros = R_cv_to_ros @ R_cv
        quat = self.rotation_matrix_to_quaternion(R_ros)

        # OpenCV → ROS translation
        tvec = tvec.flatten()
        position = np.array([
            tvec[2],
        -tvec[0],
        -tvec[1]
        ])

        return {
            'position': position,
            'orientation': quat
        }

    def rotation_matrix_to_quaternion(self, R):
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2,1]-R[1,2])*s
            y = (R[0,2]-R[2,0])*s
            z = (R[1,0]-R[0,1])*s
        elif R[0,0]>R[1,1] and R[0,0]>R[2,2]:
            s = 2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2])
            w=(R[2,1]-R[1,2])/s;x=0.25*s;y=(R[0,1]+R[1,0])/s;z=(R[0,2]+R[2,0])/s
        elif R[1,1]>R[2,2]:
            s=2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2])
            w=(R[0,2]-R[2,0])/s;x=(R[0,1]+R[1,0])/s;y=0.25*s;z=(R[1,2]+R[2,1])/s
        else:
            s=2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1])
            w=(R[1,0]-R[0,1])/s;x=(R[0,2]+R[2,0])/s;y=(R[1,2]+R[2,1])/s;z=0.25*s
        return [x,y,z,w]

    def publish_detection(self, tag_id, pose):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'apriltag_{tag_id}'
        msg.pose.position.x = float(pose['position'][0])
        msg.pose.position.y = float(pose['position'][1])
        msg.pose.position.z = float(pose['position'][2])
        msg.pose.orientation.x = pose['orientation'][0]
        msg.pose.orientation.y = pose['orientation'][1]
        msg.pose.orientation.z = pose['orientation'][2]
        msg.pose.orientation.w = pose['orientation'][3]
        self.detection_pub.publish(msg)
        if self.publish_tf_flag:
            self.publish_tf(tag_id, pose)

    def publish_tf(self, tag_id, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.camera_frame
        t.child_frame_id = f'apriltag_{tag_id}'
        t.transform.translation.x = float(pose['position'][0])
        t.transform.translation.y = float(pose['position'][1])
        t.transform.translation.z = float(pose['position'][2])
        t.transform.rotation.x = pose['orientation'][0]
        t.transform.rotation.y = pose['orientation'][1]
        t.transform.rotation.z = pose['orientation'][2]
        t.transform.rotation.w = pose['orientation'][3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    detector = AprilTagDetector()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()