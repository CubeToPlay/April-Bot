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
                    image = cv2.resize(image, (300, 300))
                    _, image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY)
                    rotations = [
                        image,
                        cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE),
                        cv2.rotate(image, cv2.ROTATE_180),
                        cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    ]
                    self.tags[tag_id] = {'image': image, 'rotations': rotations}
                    self.get_logger().info(f'Loaded image for tag ID {tag_id}')
        self.get_logger().info(f'Loaded {len(self.tags)} tag images')

    def load_tag36h11_codes(self):
        codes = {}
        for tag_id, data in self.tags.items():
            bits = self.extract_inner_bits(data['image'])
            codes[tag_id] = bits
        return codes
    def extract_inner_bits(img):
        size = img.shape[0]
        border = int(size * 0.25)  # 25% border
        inner = img[border:-border, border:-border]
        grid = cv2.resize(inner, (4,4), interpolation=cv2.INTER_AREA)
        bits = (grid < 128).astype(int)  # black=1, white=0
        return bits.flatten()
    
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
    def prune_tracks(self, timeout=0.5):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.tag_tracks = {tid: tr for tid, tr in self.tag_tracks.items() if now - tr['last_seen'] < timeout}

    def order_points(self, pts):
        rect = np.zeros((4,2), dtype="float32")
        s = pts.sum(axis=1)
        diff = np.diff(pts, axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    def extract_bit_grid_from_image(self, img):
        size = img.shape[0]
        border = int(size * 0.15)
        inner = img[border:-border, border:-border]
        grid = cv2.resize(inner, (6, 6), interpolation=cv2.INTER_AREA)
        bits = (grid > 127).astype(np.uint8)
        return bits

    def decode(self, warped, tag_size=6):
        bits = np.zeros((tag_size, tag_size), dtype=int)
        cell_size = warped.shape[0] // tag_size
        for i in range(tag_size):
            for j in range(tag_size):
                cell = warped[i*cell_size:(i+1)*cell_size, j*cell_size:(j+1)*cell_size]
                bits[i, j] = 1 if np.mean(cell) < 128 else 0

        # Inner 4x4 grid (skip border)
        inner_bits = bits[1:-1, 1:-1].flatten()
        
        # Convert 16 bits to integer
        tag_id = 0
        for i, b in enumerate(inner_bits):
            if b:
                tag_id |= (1 << i)
        return int(tag_id)
    
    def match_tag_bits(self, bits):
        for tag_id, template_bits in self.valid_tag_codes.items():
            if np.array_equal(bits, template_bits):
                return tag_id
        return None

    def find_apriltags_contours(self, image, min_size=20, tag_size=6):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, 11, 2)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detected_tags = []
        for cnt in contours:
            epsilon = 0.05 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                quad = approx.reshape(4,2)
                w = np.linalg.norm(quad[0] - quad[1])
                h = np.linalg.norm(quad[0] - quad[3])
                if w < min_size or h < min_size:
                    continue
                quad = self.order_points(quad)
                warp_size = 200
                dst = np.array([[0,0],[warp_size-1,0],[warp_size-1,warp_size-1],[0,warp_size-1]], dtype=np.float32)
                M = cv2.getPerspectiveTransform(quad, dst)
                warped = cv2.warpPerspective(gray, M, (warp_size, warp_size))
                tag_bits = self.extract_inner_bits(warped)
                tag_id = self.match_tag_bits(tag_bits)
                if tag_id is not None:
                    detected_tags.append({'id': tag_id, 'corners': quad, 'center': np.mean(quad, axis=0)})
        return detected_tags

    def detect_apriltags(self, img):
        return self.find_apriltags_contours(img)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        detected_tags = self.detect_apriltags(cv_image)
        vis_image = cv_image.copy()
        detections_array = AprilTagDetectionArray()
        detections_array.header.stamp = self.get_clock().now().to_msg()
        detections_array.header.frame_id = self.camera_frame
        self.prune_tracks()

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
        half_size = self.tag_size / 2
        object_points = np.array([[-half_size, -half_size, 0],
                                  [half_size, -half_size, 0],
                                  [half_size, half_size, 0],
                                  [-half_size, half_size, 0]], dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(object_points, corners.astype(np.float32),
                                           self.camera_matrix, self.dist_coeffs,
                                           flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            return None
        R, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(R)
        return {'position': tvec.flatten(), 'orientation': quat}

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
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()