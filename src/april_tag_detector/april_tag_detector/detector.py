import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Pose
from std_msgs.msg import Header
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import cv2
import numpy as np
import math
import os

from april_tag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('detector')
        
        self.declare_parameter('tag_size', 0.16)
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

        self.max_hamming = 4

        self.min_confirm_time = 0.2  # seconds
        self.max_center_jump = 50.0  # pixels

        self.tag_tracks = {}  
        # tag_id -> {
        #   'first_seen': time,
        #   'last_seen': time,
        #   'last_center': np.array([x,y]),
        #   'count': int
        # }

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

    def load_tags(self):
        """Load AprilTag images from directory
        AprilTag format: tag36h11-<id>.png
        """
        # Check if directory exists
        if not os.path.exists(self.tag_dir):
            self.get_logger().warn(f'Template directory not found: {self.tag_dir}')
            return
        # It is looking for files that match the pattern: tag36h11-<id>.png
        # For each tag image, it extracts the id from the filename, loads it in grayscale, resizes it to 300 x 300, generates 4 rotations of the image, and stores it in the self.tags dictionary
        for filename in os.listdir(self.tag_dir):
            if filename.startswith('tag36h11-') and (filename.endswith('.png')):
                # Extract ID from filename
                try:
                    tag_id = int(filename.split('-')[1].split('.')[0])
                except:
                    self.get_logger().warn(f'Could not parse tag ID from: {filename}')
                    continue
                filepath = os.path.join(self.tag_dir, filename)
                image = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)

                if image is not None:
                    image = cv2.resize(image, (300, 300))
                    _, image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY)

                    rotations = [
                        image,
                        cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE),  # 90 degrees
                        cv2.rotate(image, cv2.ROTATE_180),  # 180 degrees
                        cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)  # 270 degrees
                    ]

                    self.tags[tag_id] = {'image': image, 'rotations': rotations}

                    self.get_logger().info(f'Loaded image for tag ID {tag_id}')
        
        self.get_logger().info(f'Loaded {len(self.tags)} tag images')

    def load_tag36h11_codes(self):
        codes = {}
        for tag_id, data in self.tags.items():
            bits = self.extract_bit_grid_from_image(data['image'])
            codes[tag_id] = bits
            
            # Debug: print the bit pattern
            self.get_logger().info(f'Tag {tag_id} bits:\n{bits}')
        
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
    
    def update_tag_track(self, tag_id, center):
        now = self.get_clock().now().nanoseconds * 1e-9

        if tag_id not in self.tag_tracks:
            self.tag_tracks[tag_id] = {
                'first_seen': now,
                'last_seen': now,
                'last_center': center,
                'count': 1
            }
            return False  # not confirmed yet

        track = self.tag_tracks[tag_id]

        # Reject if it jumped too much (likely false match)
        if np.linalg.norm(center - track['last_center']) > self.max_center_jump:
            self.tag_tracks[tag_id] = {
                'first_seen': now,
                'last_seen': now,
                'last_center': center,
                'count': 1
            }
            return False

        track['last_seen'] = now
        track['last_center'] = center
        track['count'] += 1

        return (now - track['first_seen']) >= self.min_confirm_time
    
    def prune_tracks(self, timeout=0.5):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.tag_tracks = {
            tid: tr for tid, tr in self.tag_tracks.items()
            if now - tr['last_seen'] < timeout
        }
    
    def check_tag_completeness(self, corners, img_shape):
        """Check if all 4 corners are well within the image bounds"""
        margin = 10  # pixels from edge
        height, width = img_shape[:2]
        
        for corner in corners:
            x, y = corner
            # Check if corner is too close to image edge
            if (x < margin or x > width - margin or 
                y < margin or y > height - margin):
                return False
        
        return True
    
    def matches_known_apriltag(self, bits):
        """
        bits: 6x6 numpy array of {0,1}
        Returns (tag_id, rotation) or (None, None)
        """
        if bits is None or bits.shape != (6,6):
            return None, None
        self.get_logger().info(f'Detected bits:\n{bits}', throttle_duration_sec=5.0)

        def rotate(bits, k):
            return np.rot90(bits, k)

        def hamming(a, b):
            return np.count_nonzero(a != b)

        best_id = None
        best_rot = None
        best_dist = 999

        for tag_id, tag_bits in self.valid_tag_codes.items():
            self.get_logger().info(f'Comparing to Tag {tag_id} bits:\n{tag_bits}', once=True)
            for rot in range(4):
                rotated = rotate(bits, rot)
                dist = hamming(rotated, tag_bits)

                if dist < best_dist:
                    best_dist = dist
                    best_id = tag_id
                    best_rot = rot

                    if dist == 0:
                        return best_id, best_rot  # exact match

        if best_dist <= self.max_hamming:
            return best_id, best_rot

        return None, None
    def has_strong_cell_contrast(self, warped_bin):
        inner = warped_bin[warped_bin.shape[0]//7:-warped_bin.shape[0]//7,
                        warped_bin.shape[1]//7:-warped_bin.shape[1]//7]

        grid = cv2.resize(inner, (6, 6), interpolation=cv2.INTER_AREA)
        vals = grid.flatten()

        return np.std(vals) > 40

    def validate_tag_quality(self, gray_img, corners):
        """Combined validation for tag quality"""
        # Check if tag is complete (not cut off by image edge)
        return self.check_tag_completeness(corners, gray_img.shape)
    
    def has_valid_border(self, warped_bin, border_frac=0.12, min_ratio=0.70):
        h, w = warped_bin.shape
        b = int(min(h, w) * border_frac)

        borders = np.concatenate([
            warped_bin[:b, :].ravel(),
            warped_bin[-b:, :].ravel(),
            warped_bin[:, :b].ravel(),
            warped_bin[:, -b:].ravel()
        ])

        white_ratio = np.mean(borders == 255)
        return white_ratio > min_ratio
    
    def border_is_continuous(self, warped_bin):
        edges = cv2.Canny(warped_bin, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False

        largest = max(contours, key=cv2.contourArea)
        peri = cv2.arcLength(largest, True)

        # partial tags have broken perimeters
        return peri > 0.9 * (4 * warped_bin.shape[0])
    
    def extract_bit_grid_from_image(self, img):
        """Extract 6x6 bit grid from a binary image (works for both templates and warped images)"""
        size = img.shape[0]
        border = int(size * 0.15)  # Remove white border
        inner = img[border:-border, border:-border]
        
        # Resize to 6x6 grid
        grid = cv2.resize(inner, (6, 6), interpolation=cv2.INTER_AREA)
        
        # Threshold to get bits
        bits = (grid > 127).astype(np.uint8)
        return bits

    def extract_bit_grid(self, gray, corners, grid_size=6):
        size = 300
        dst = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(corners, dst)

        warped = cv2.warpPerspective(
            gray, M, (size, size), flags=cv2.INTER_NEAREST
        )

        _, warped_bin = cv2.threshold(warped, 127, 255, cv2.THRESH_BINARY  + cv2.THRESH_OTSU)
        # if not self.has_valid_border(warped_bin):
        #     return None

        # # if not self.border_is_continuous(warped_bin):
        # #     return None
        # if not self.has_strong_cell_contrast(warped_bin):
        #     return None
        # Decode bits
        bits = self.extract_bit_grid_from_image(warped_bin)
        return bits
    
    def detect_apriltags(self, img):
        """Detects all AprilTags in the given image, using two methods in order to detect corners of AprilTags"""
        # Convert the given image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detect corners using Shi-Tomasi method
        corners_detected = cv2.goodFeaturesToTrack(
            blurred,
            maxCorners=500, # Find up to 500 corners
            qualityLevel=0.005, # Minimum quality threshold
            minDistance=5, # Minimum pixel distance between corners
            blockSize=7 # Size of neighborhood for corner detection
        )
        
        if corners_detected is None:
            self.get_logger().warn('No corners detected!', throttle_duration_sec=2.0)
            return []
        
        corners_detected = corners_detected.reshape(-1, 2)

        # Alternate corner detector to make corner detection more accurate
        harris = cv2.cornerHarris(blurred, blockSize=2, ksize=3, k=0.04)
        harris = cv2.dilate(harris, None)

        harris_threshold = 0.01 * harris.max()
        harris_corners = np.argwhere(harris > harris_threshold)
        harris_corners = harris_corners[:, [1, 0]].astype(np.float32)

        # Combine all corners and use that combination to find quadrilaterals from the corners, which find the detected tags
        all_corners = np.vstack([corners_detected, harris_corners])

        detected_tags = self.find_quadrilaterals_from_corners(gray, all_corners)
        
        return detected_tags
    
    def find_quadrilaterals_from_corners(self, gray, corners):
        """Find quadrilaterals by grouping nearby corners"""
        detected_tags = []

        # Tries different window sizes in order to detect tags at different distances (small windows: far away tags, large windows: close tags)
        for window_size in [50, 80, 120, 160, 200]:
            # Grid-based clustering: Divides images into overlapping grid regions, where each region is checked for corner clusters
            grid_x = np.arange(0, gray.shape[1], window_size)
            grid_y = np.arange(0, gray.shape[0], window_size)
            

            # For each grid cell, it finds all corners in the window and if more than 4 corners are found, select_quadrilateral_corners is called to form a 4-sided shape, which is validated with validate_quadrilateral.
            # The corners are consistently ordered with order_points
            # The images are matched against the images given in self.tags using match_image 
            for x in grid_x:
                for y in grid_y:
                    # Find corners in this region
                    mask_x = (corners[:, 0] >= x) & (corners[:, 0] < x + window_size)
                    mask_y = (corners[:, 1] >= y) & (corners[:, 1] < y + window_size)
                    region_corners = corners[mask_x & mask_y]
                    self.get_logger().info(f'Checking window at ({x},{y}), found {len(region_corners)} corners', 
                       throttle_duration_sec=5.0)

                    if len(region_corners) >= 4:
                        # Take the 4 most extreme corners
                        quad_corners = self.select_quadrilateral_corners(region_corners)

                        if quad_corners is not None:
                            self.get_logger().info('Found a quadrilateral!', throttle_duration_sec=2.0)
                            # Verify this looks like a tag
                            if self.validate_quadrilateral(quad_corners):
                                # Order corners
                                ordered_corners = self.order_points(quad_corners)
                                
                                # if not self.validate_tag_quality(gray, ordered_corners):
                                #     continue
                                # Match against tag images
                                bits = self.extract_bit_grid(gray, ordered_corners)
                                if bits is None:
                                    self.get_logger().warn('Bit extraction failed!', throttle_duration_sec=2.0)
                                    continue
                                else:
                                    self.get_logger().info(f'Extracted bits successfully', throttle_duration_sec=2.0)
                                tag_id, rotation = self.matches_known_apriltag(bits)

                                if tag_id is None:
                                    continue
                                self.get_logger().info(f'Match result: ID={tag_id}, rotation={rotation}', throttle_duration_sec=2.0)

                                if tag_id is not None:
                                    # Check if the tag has already been detected
                                    is_duplicate = False
                                    for existing_tag in detected_tags:
                                        center_dist = np.linalg.norm(
                                            np.mean(ordered_corners, axis=0) - 
                                            existing_tag['center']
                                        )
                                        if center_dist < 20:  # pixels
                                            is_duplicate = True
                                            break
                                    if not is_duplicate:
                                        detected_tags.append({
                                            'id': tag_id,
                                            'corners': ordered_corners,
                                            'center': np.mean(ordered_corners, axis=0),
                                            'rotation': rotation
                                        })
        return detected_tags
    
    def select_quadrilateral_corners(self, corners):
        """Select 4 corners that form a quadrilateral"""
        if len(corners) == 4:
            return corners
        
        if len(corners) > 4:
            hull = cv2.convexHull(corners.astype(np.float32))
            hull = hull.reshape(-1, 2)
            
            if len(hull) >= 4:
                # Take 4 corners with largest angles
                if len(hull) == 4:
                    return hull
                else:
                    # Reduce the number of points to 4 if more than 4 are in the hull
                    peri = cv2.arcLength(hull, True)
                    approx = cv2.approxPolyDP(hull, 0.02 * peri, True)
                    if len(approx) == 4:
                        return approx.reshape(4, 2)
        
        return None
    
    def validate_quadrilateral(self, corners):
        """Check if corners form a valid quadrilateral and that false positives are filtered out."""
        if len(corners) != 4:
            return False
        
        area = cv2.contourArea(corners.astype(np.float32))
        if area < 300:
            return False
        
        # Get the lengths of each side of the quadrilateral
        ordered = self.order_points(corners)
        width1 = np.linalg.norm(ordered[0] - ordered[1])
        width2 = np.linalg.norm(ordered[2] - ordered[3])
        height1 = np.linalg.norm(ordered[0] - ordered[3])
        height2 = np.linalg.norm(ordered[1] - ordered[2])

        avg_width = (width1 + width2) / 2
        avg_height = (height1 + height2) / 2
        
        if avg_width == 0 or avg_height == 0:
            return False
        
        # Getting the largest length divided by the smallest length to determine how many times larger the largest length is.
        aspect_ratio = max(avg_width, avg_height) / min(avg_width, avg_height)

        # Tags should be roughly square
        if aspect_ratio > 3.0:
            return False
        
        return True
    
    def order_points(self, pts):
        """Order corner points: top-left, top-right, bottom-right, bottom-left"""
        pts = pts.reshape(4, 2)
        rect = np.zeros((4, 2), dtype="float32")
        
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # Top-left
        rect[2] = pts[np.argmax(s)]  # Bottom-right
        
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # Top-right
        rect[3] = pts[np.argmax(diff)]  # Bottom-left
        
        return rect
    def decode(self, warped_bin):
        size = warped_bin.shape[0]
        border = size // 7

        inner = warped_bin[border:-border, border:-border]
        grid = cv2.resize(inner, (6, 6), interpolation=cv2.INTER_AREA)

        bits = (grid > 127).astype(np.uint8)

        return bits
    
    # def match_image(self, gray_img, corners):
    #     """Match warped tag with given tag images"""

    #     if not self.tags:
    #         return self.decode_tag(gray_img, corners), 0
        
    #     size = 300
    #     dst = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype="float32")
    #     M = cv2.getPerspectiveTransform(corners, dst)
    #     warped = cv2.warpPerspective(gray_img, M, (size, size))
        
    #     # Threshold
    #     _, warped_bin = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
    #     # Try to match against all images and rotations
    #     best_match_score = -1
    #     best_tag_id = None
    #     best_rotation = 0

    #     for tag_id, image_data in self.tags.items():
    #         for rot_idx, rotated_template in enumerate(image_data['rotations']):
    #             score = self.compare_images(warped_bin, rotated_template)

    #             if score > best_match_score:
    #                 best_match_score = score
    #                 best_tag_id = tag_id
    #                 best_rotation = rot_idx
        
    #     # Threshold for accepting a match
    #     if best_match_score > 0.7:
    #         return best_tag_id, best_rotation
        
    #     return None, None
    
    # def compare_images(self, img1, img2):
    #     """Determine how similar two images are to one another"""
    #     if img1.shape != img2.shape:
    #         return 0.0
        
    #     diff = cv2.bitwise_xor(img1, img2)

    #     total_pixels = img1.shape[0] * img1.shape[1]
    #     different_pixels = np.count_nonzero(diff)
    #     similarity = 1.0 - (different_pixels / total_pixels)
        
    #     return similarity
    
    # def decode_tag(self, gray_img, corners):
    #     """Backup decode incase a given AprilTag does not match any given AprilTag values"""
    #     size = 300
    #     dst = np.array([[0, 0], [size, 0], [size, size], [0, size]], dtype="float32")
        
    #     M = cv2.getPerspectiveTransform(corners, dst)
    #     warped = cv2.warpPerspective(gray_img, M, (size, size))

    #     _, tag_bin = cv2.threshold(warped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
    #     # Extract inner grid (remove border)
    #     inner = tag_bin[int(size/7):int(6*size/7), int(size/7):int(6*size/7)]
        
    #     # Resize to 8x8 grid
    #     grid = cv2.resize(inner, (8, 8), interpolation=cv2.INTER_AREA)
    #     _, grid_bin = cv2.threshold(grid, 127, 255, cv2.THRESH_BINARY)
        
    #     # Decode ID from inner 6x6 grid
    #     inner_grid = grid_bin[1:7, 1:7]
    #     bits = (inner_grid > 127).flatten()
        
    #     # Simple validation
    #     if bits.sum() < 5 or bits.sum() > 30:
    #         return None
        
    #     # Convert to ID (using first 16 bits)
    #     tag_id = 0
    #     for i in range(min(16, len(bits))):
    #         if bits[i]:
    #             tag_id |= (1 << i)
        
    #     return tag_id % 10
    
    def estimate_pose_pnp(self, corners):
        """Estimate 3D pose using PnP"""
        if self.camera_matrix is None:
            return None
        
        # 3D points of tag corners in tag frame
        half_size = self.tag_size / 2
        object_points = np.array([
            [-half_size, -half_size, 0],
            [half_size, -half_size, 0],
            [half_size, half_size, 0],
            [-half_size, half_size, 0]
        ], dtype=np.float32)
        
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            corners.astype(np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            return None
        
        # Convert rotation vector to quaternion
        R, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(R)
        
        return {
            'position': tvec.flatten(),
            'orientation': quat
        }
    
    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion [x, y, z, w]"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]
    
    def publish_detection(self, tag_id, pose):
        """Publish detection as PoseStamped"""
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
        
        # Publish TF
        if self.publish_tf_flag:
            self.publish_tf(tag_id, pose)

    def publish_tf(self, tag_id, pose):                                                                             
        """Publish TF transform for tag"""
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

    def draw_detection(self, img, tag_id, corners, pose, rotation=0):
        """Draw detection on image"""
        # Draw quadrilateral
        corners_int = corners.astype(int)
        cv2.polylines(img, [corners_int], True, (0, 255, 0), 2)
        
        # Draw ID
        center = np.mean(corners, axis=0).astype(int)
        cv2.putText(img, f"ID: {tag_id}", tuple(center),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw rotation info
        rot_text = f"Rot: {rotation * 90}deg"
        cv2.putText(img, rot_text, (center[0], center[1] + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Draw distance
        distance = np.linalg.norm(pose['position'])
        cv2.putText(img, f"{distance:.2f}m", (center[0], center[1] + 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # Draw axes
        if self.camera_matrix is not None:
            self.draw_axes(img, corners, pose)
        
        return img
    
    def draw_axes(self, img, corners, pose):
        """Draw 3D coordinate axes"""
        # Axis length
        axis_length = self.tag_size / 2
        
        # 3D points for axes
        axis_points = np.float32([
            [0, 0, 0],
            [axis_length, 0, 0],  # X-axis
            [0, axis_length, 0],  # Y-axis
            [0, 0, -axis_length]  # Z-axis (out of tag)
        ])
        
        # Convert pose to rvec, tvec
        R = self.quaternion_to_rotation_matrix(pose['orientation'])
        rvec, _ = cv2.Rodrigues(R)
        tvec = pose['position'].reshape(3, 1)
        
        # Project to image
        img_points, _ = cv2.projectPoints(
            axis_points, rvec, tvec,
            self.camera_matrix, self.dist_coeffs
        )
        img_points = img_points.reshape(-1, 2).astype(int)
        
        # Draw axes
        origin = tuple(img_points[0])
        cv2.line(img, origin, tuple(img_points[1]), (0, 0, 255), 3)  # X = Red
        cv2.line(img, origin, tuple(img_points[2]), (0, 255, 0), 3)  # Y = Green
        cv2.line(img, origin, tuple(img_points[3]), (255, 0, 0), 3)  # Z = Blue
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion [x,y,z,w] to rotation matrix"""
        x, y, z, w = q
        return np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
    
    def image_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        # Detect tags
        detected_tags = self.detect_apriltags(cv_image)
        
        # Create visualization
        vis_image = cv_image.copy()
        
        # Create array message for all detections
        detections_array = AprilTagDetectionArray()
        detections_array.header.stamp = self.get_clock().now().to_msg()
        detections_array.header.frame_id = self.camera_frame
        
        self.prune_tracks()

        for tag in detected_tags:
            tag_id = tag['id']
            center = tag['center']

            confirmed = self.update_tag_track(tag_id, center)

            if not confirmed:
                continue
            corners = tag['corners']
            rotation = tag.get('rotation', 0)
            
            # Estimate pose
            pose = self.estimate_pose_pnp(corners)
            
            if pose is not None:
                # Publish individual detection (legacy support)
                self.publish_detection(tag_id, pose)
                
                # Create detection message for array
                detection = AprilTagDetection()
                detection.id = tag_id
                
                # Set pose
                detection.pose.position.x = float(pose['position'][0])
                detection.pose.position.y = float(pose['position'][1])
                detection.pose.position.z = float(pose['position'][2])
                detection.pose.orientation.x = pose['orientation'][0]
                detection.pose.orientation.y = pose['orientation'][1]
                detection.pose.orientation.z = pose['orientation'][2]
                detection.pose.orientation.w = pose['orientation'][3]
                
                # Set corners (flatten to [x0,y0,x1,y1,x2,y2,x3,y3])
                detection.corners = corners.flatten().tolist()
                
                # Set rotation
                detection.rotation = rotation
                
                # Set size
                detection.size = self.tag_size
                
                # Set decision margin (placeholder - could be computed from template matching score)
                detection.decision_margin = 0.8
                
                # Set hamming distance (placeholder - 0 means perfect match)
                detection.hamming = 0
                
                # Add to array
                detections_array.detections.append(detection)
                
                # Draw on image
                vis_image = self.draw_detection(vis_image, tag_id, corners, pose, rotation)
                
                self.get_logger().info(
                    f'Tag {tag_id} (rot={rotation*90}°): ({pose["position"][0]:.2f}, '
                    f'{pose["position"][1]:.2f}, {pose["position"][2]:.2f})',
                    throttle_duration_sec=1.0
                )
        
        # Publish array of all detections
        if len(detections_array.detections) > 0:
            self.detections_array_pub.publish(detections_array)
            self.get_logger().info(
                f'Published {len(detections_array.detections)} detections in array', 
                throttle_duration_sec=1.0
            )
        
        # Publish visualization
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            self.visualization_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish visualization: {e}')

def main(args=None):
    rclpy.init(args=args)
    detector = AprilTagDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()