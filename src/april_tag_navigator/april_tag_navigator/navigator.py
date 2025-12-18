import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Int32, Bool
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from slam_toolbox.srv import SerializePoseGraph
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import math
from enum import Enum
import json
import os
import heapq
from collections import deque

from april_tag_msgs.msg import AprilTagDetection, AprilTagDetectionArray

class NavigationState(Enum):
    """Indicates what the robot is currently doing"""
    IDLE = 0
    """Robot doesn't have task, will remain where it is untill it receives a new goal id"""
    PLANNING = 1
    """Robot is currently planning the path with A* Algorithm from where it is to the target location"""
    NAVIGATING = 2
    """Robot is moving along the planned path to the goal"""
    TRACKING = 3
    """Robot sees the goal april tag and is now moving directly towards it (not following the path exactly)"""
    REACHED = 4
    """Robot has reached the april tag and will be waiting for next instruction"""
    RECOVERY = 5
    """Robot got too close to an obstacle and has to move away from it"""
    SCANNING = 6
    """Robot will spin when it reaches a frontier to ensure that it sees all walls"""

class AprilTagNavigator(Node):
    """Uses A* algorithm for path planning to get the robot to move to the given goal id. If it knows where the tag is, it will go directly there, otherwise it will begin searching for the april tag in unexplored locations"""
    def __init__(self):
        super().__init__('navigator')

        # Parameters
        self.declare_parameter('approach_distance', 0.3)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('tag_database_path', 'discovered_tags.json')
        self.declare_parameter('frontier_grid_size', 0.5)
        self.declare_parameter('cancel_command', 11)
        self.declare_parameter('min_wall_distance', 0.35)   # meters
        self.declare_parameter('critical_distance', 0.20)  # emergency stop
        self.declare_parameter('recover_back_speed', 0.3)
        self.declare_parameter('recovery_back_time', 0.5)
        self.declare_parameter('recovery_turn_speed', 0.2)
        

        
        self.approach_distance = self.get_parameter('approach_distance').value
        """How close the robot has to be to count the tag as approached"""
        self.linear_speed = self.get_parameter('linear_speed').value
        """Max robot linear speed"""
        self.angular_speed = self.get_parameter('angular_speed').value
        """Max robot angular speed"""
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        """How close the robot has to be to the waypoint to count as being there"""
        self.tag_database_path = self.get_parameter('tag_database_path').value
        """Gives the path to the saved tags for the given world"""
        self.frontier_grid_size = self.get_parameter('frontier_grid_size').value
        """This is the resolution of the exploration grid (dividing the world into cells for tracking the areas that have not been explored)"""
        self.cancel_command = self.get_parameter('cancel_command').value
        """When goal_id = 11, it means that the current search should be cancelled and the robot should be idle."""

        self.min_wall_distance = self.get_parameter('min_wall_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value

        self.recover_back_speed = self.get_parameter('recover_back_speed').value
        self.recovery_back_time = self.get_parameter('recovery_back_time').value
        self.recovery_turn_speed = self.get_parameter('recovery_turn_speed').value
        self.recovery_phase = 0
        self.recovery_start_time = None

        self.scan_start_yaw = None
        self.last_yaw = None
        self.total_yaw = 0.0
        self.scan_target = 2 * math.pi
        self.scan_speed = 0.8

        # TF2 for SLAM localization
        try:
            use_sim = self.get_parameter('use_sim_time').value
            self.get_logger().info(f'use_sim_time: {use_sim}')
        except:
            self.get_logger().warning('use_sim_time not set!')
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        """Stores the last set of transformations"""
        self.tf_listener = TransformListener(self.tf_buffer, self)
        """Receives and stores transforms that were published on the /tf topic"""

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        """Subscribes to LiDAR data. """
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Critical for /map
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        """Subscribes to the SLAM generated map. Updated periodically as the robot explores the environment"""
        self.goal_sub = self.create_subscription(Int32, '/goal_id', self.goal_callback, 10)
        """Subscribes to goal inputs. The goal id is published here."""
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray, '/april_tags', self.detection_callback, 10
        )
        """Subscribes to AprilTag detections. The complete list of identified AprilTags in view are sent here. It contains ID and pose
        AprilTagDetectionArray:
            std_msgs/Header header
            AprilTagDetection[] detections
        AprilTagDetection:
            int32 id
            geometry_msgs/Pose pose
            float32 decision_margin
            int32 hamming
            float32[8] corners
            float32 size 
        """

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        """Publishes velocity commands to robot."""
        self.path_pub = self.create_publisher(Marker, '/planned_path', 10)
        """Publishes A* path for visualization in RViz. Used for debugging path."""
        self.reach_goal_pub = self.create_publisher(Bool, '/reach_goal', 10)
        """Publishes boolean if the robot has reached the given goal"""
        self.marker_pub = self.create_publisher(MarkerArray, '/apriltag_markers', 10)

        # State
        self.state = NavigationState.IDLE
        """Robot's current state"""
        self.robot_pose = None 
        """Gotten from SLAM"""
        self.target_tag_id = None
        """The current goal id that the robot is going to"""
        self.target_tag_visible = False
        """If the target AprilTag is currently in view"""
        self.target_tag_distance = None
        """Distance from target AprilTag"""
        self.target_tag_angle = None
        """Angle to target AprilTag in degrees"""
        
        # Map data
        self.map_data = None
        """2D numpy array where each cell represents cell in map. Each cell is 0-100 (occupancy probability) or -1 (unknown)"""
        self.map_resolution = 0.05
        """Meters per cell (0.05 = 5cm cells)"""
        self.map_origin = {'x': 0.0, 'y': 0.0}
        """World coordinates of map's bottom-left corner"""
        self.map_width = 0
        self.map_height = 0
        
        # Path planning
        self.current_path = []
        """Lists the waypoints from A* path planning algorithm that the robot will move through to get to target location"""
        self.path_index = 0
        """Which waypoint the robot is currently moving toward"""
        
        # Tag database
        self.discovered_tags = {}
        """Tag locations in map frame
        {tag_id: {'x': float, 'y': float, 'z': float}}
        """
        self.current_detections = {}
        """Lists the tag ids that are seen in the current image frame"""

        self.frontier_target = None
        """Current exploration destination: {'x': float, 'y': float}"""
        
        # LiDAR
        self.laser_ranges = None
        """Array of distance measurements from LiDAR"""
        self.laser_msg = None

        self.tracked_goal = None

        # Timers
        self.map_wait_timer = self.create_timer(1.0, self.wait_for_map_timer)
        """Wait for /map to begin publishing"""

        self.pose_timer = None
        """Timer to update the robot pose every 0.2 secconds"""

        self.nav_timer = None
        """Timer to run the navigation loop to move the robot"""

        self.marker_timer = None
        
        # Load database
        self.load_tag_database()
    
    def map_ready(self):
        if self.map_data is None:
            return False

        if len(self.map_data) == 0:
            return False
        try:
            return self.tf_buffer.can_transform(
                'map',
                'base_footprint',
                Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception:
            return False
    def wait_for_map_timer(self):
        if not self.map_ready():
            self.get_logger().info('Waiting for map...')
            return

        self.get_logger().info('Map ready. Starting navigation.')
        try:
            # Try to get the transform
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                Time(),  # Get latest
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            self.get_logger().info(
                f'Map received: {self.map_width}x{self.map_height} cells'
            )
            self.get_logger().info(
                f'TF ready: map->base_footprint transform available'
            )
            self.get_logger().info('Starting navigation system!')
            
            # Cancel this timer and start navigation
            self.map_wait_timer.cancel()
            self.start_navigation()
            
        except TransformException as ex:
            self.get_logger().info(
                f'Waiting for map->base_footprint TF... )', 
                throttle_duration_sec=2.0
            )
            self.get_logger().debug(f'TF error: {ex}')
    
    def start_navigation(self):
        """
        Called once when the map and TF are confirmed ready.
        Starts all navigation-related timers safely.
        """

        if hasattr(self, 'navigation_started') and self.navigation_started:
            return

        self.navigation_started = True

        self.get_logger().info('Starting navigation stack')
        
        self.spin_start_time = self.get_clock().now()

        # Initialize pose tracking
        self.pose_timer = self.create_timer(
            0.2,
            self.update_robot_pose
        )

        # Main navigation loop
        self.nav_timer = self.create_timer(
            0.05,
            self.navigation_loop
        )

        self.marker_timer = self.create_timer(0.5, self.publish_tag_markers)

        # Ensure robot starts stopped
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        # Initialize state machine
        self.state = NavigationState.IDLE

    def publish_tag_markers(self):
        """Publish visualization markers for all discovered tags"""
        if not self.discovered_tags:
            return

        marker_array = MarkerArray()
        
        for tag_id, pos in self.discovered_tags.items():
            # Create text marker
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "tag_labels"
            text_marker.id = tag_id * 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = pos['x']
            text_marker.pose.position.y = pos['y']
            text_marker.pose.position.z = 0.5  # Above the cube

            text_marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            
            text_marker.scale.z = 0.2  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"Tag {tag_id}"
            
            # Create cube marker
            cube_marker = Marker()
            cube_marker.header.frame_id = "map"
            cube_marker.header.stamp = self.get_clock().now().to_msg()
            cube_marker.ns = "tag_cubes"
            cube_marker.id = tag_id * 2 + 1
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            
            cube_marker.pose.position.x = pos['x']
            cube_marker.pose.position.y = pos['y']
            cube_marker.pose.position.z = 0.15
            cube_marker.pose.orientation.w = 1.0
            
            cube_marker.scale.x = 0.2
            cube_marker.scale.y = 0.2
            cube_marker.scale.z = 0.3
            
            # Color based on whether it's the target
            if tag_id == self.target_tag_id:
                cube_marker.color.r = 1.0
                cube_marker.color.g = 0.0
                cube_marker.color.b = 0.0
            else:
                cube_marker.color.r = 0.0
                cube_marker.color.g = 1.0
                cube_marker.color.b = 0.0
            cube_marker.color.a = 0.8
            
            marker_array.markers.append(text_marker)
            marker_array.markers.append(cube_marker)
        
        self.marker_pub.publish(marker_array)

    def check_obstacle_ahead(self):
        """
        Returns:
            critical (bool): emergency stop required
            warning (bool): slow down / steer away
            min_dist (float): closest obstacle ahead
        """
        if self.laser_msg is None:
            return False, False, float('inf')

        msg = self.laser_msg
        ranges = np.array(msg.ranges)

        # Filter invalid values
        valid = np.isfinite(ranges)
        ranges = ranges[valid]

        if len(ranges) == 0:
            return False, False, float('inf')

        # Compute angles for each range
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        angles = angles[valid]

        # Front ±30 degrees
        front_mask = np.abs(angles) < math.radians(30)

        front_ranges = ranges[front_mask]

        if len(front_ranges) == 0:
            return False, False, float('inf')

        min_dist = np.min(front_ranges)

        critical = min_dist < self.critical_distance
        warning = min_dist < self.min_wall_distance

        return critical, warning, min_dist

    def goal_callback(self, msg):
        """Receive target tag ID"""
        self.target_tag_id = msg.data
        self.state = NavigationState.PLANNING
        # If cancel command is given, the robot should become idle.
        if self.target_tag_id == self.cancel_command:
            self.state = NavigationState.IDLE
            self.tracked_goal = None
            self.current_path = []
            self.recovery_phase = 0
        elif self.target_tag_id in self.discovered_tags:
            self.get_logger().info(f'Planning path to KNOWN tag {self.target_tag_id}')
        else:
            self.get_logger().info(f'Searching for UNKNOWN tag {self.target_tag_id}')


    def detection_callback(self, msg):
        """Receive AprilTags detection"""

        if self.map_data is None or self.robot_pose is None:
            return
        # Need to clear out all perviously seen tags before listing the newly seen ones
        self.current_detections = {}
        seen_target = False
        for detection in msg.detections:
            # Get tag_id and then add it to the current_detections with its pose
            tag_id = detection.id
            tag_frame = f"apriltag_{tag_id}"
            pose = detection.pose
            self.current_detections[tag_id] = {
                'pose': pose
            }

            # Get robot orientation
            q = self.robot_pose['orientation']
            robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            # Angle from camera Z axis (forward)
            angle_camera = math.atan2(pose.position.x, pose.position.z)
            
            # Transform to map frame
            # Assuming camera points in same direction as robot
            tag_angle_map = robot_yaw + angle_camera

            try:
                transform = self.tf_buffer.lookup_transform(
                'map',
                tag_frame,
                Time(),                      # ← latest transform
                timeout=Duration(seconds=0.2)
            )
                tag_x = transform.transform.translation.x  # Actual 3D position
                tag_y = transform.transform.translation.y
                if tag_id not in self.discovered_tags:
                    self.discovered_tags[tag_id] = {
                        'x': tag_x,
                        'y': tag_y,
                        'z': 0.0
                    }
                    self.get_logger().info(
                        f'Discovered Tag {tag_id} at map coordinates '
                        f'({tag_x:.2f}, {tag_y:.2f})'
                    )
                else:
                    # Smooth update using moving average
                    alpha = 0.3
                    old = self.discovered_tags[tag_id]
                    self.discovered_tags[tag_id] = {
                        'x': alpha * tag_x + (1 - alpha) * old['x'],
                        'y': alpha * tag_y + (1 - alpha) * old['y'],
                        'z': alpha * 0.0 + (1 - alpha) * old['z']
                    }
                
                self.get_logger().info(
                    f'Tag {tag_id}: map=({self.discovered_tags[tag_id]["x"]:.2f}, '
                    f'{self.discovered_tags[tag_id]["y"]:.2f})',
                    throttle_duration_sec=2.0
                )
                
            except TransformException as ex:
                # Fallback: Calculate from robot pose + camera frame detection
                # This is less accurate but works when TF is unavailable
                self.get_logger().debug(f'TF failed for tag {tag_id}, using fallback: {ex}')
                
                
                # Tag position in camera frame
                # Camera frame: Z forward, X right, Y down
                # We need: distance and angle
                tag_dist_camera = math.sqrt(
                    pose.position.x**2 + 
                    pose.position.y**2 + 
                    pose.position.z**2
                )
                
                tag_x = self.robot_pose['x'] + tag_dist_camera * math.cos(tag_angle_map)
                tag_y = self.robot_pose['y'] + tag_dist_camera * math.sin(tag_angle_map)
                
                if tag_id not in self.discovered_tags:
                    self.discovered_tags[tag_id] = {
                        'x': tag_x,
                        'y': tag_y,
                        'z': 0.0
                    }
            
            # Update tracking info for target tag
            if tag_id == self.target_tag_id:
                seen_target = True
                
                # Use CAMERA FRAME for tracking (for visual servoing)
                self.target_tag_distance = math.sqrt(
                    pose.position.x**2 + 
                    pose.position.y**2 + 
                    pose.position.z**2
                )
                
                # Angle in camera frame (degrees)
                # X is left/right, Z is forward
                self.target_tag_angle = math.degrees(
                    math.atan2(pose.position.x, pose.position.z)
                )
                
                self.get_logger().info(
                    f'Tracking target {tag_id}: '
                    f'dist={self.target_tag_distance:.2f}m, '
                    f'angle={self.target_tag_angle:.1f}°',
                    throttle_duration_sec=1.0
                )
        
        self.target_tag_visible = seen_target

    def update_robot_pose(self):
        """Get robot pose from SLAM"""
        try:
            # Get the latest available transform
            # Don't specify time - just get latest
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint',
                Time(),  # Latest
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.robot_pose = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'orientation': transform.transform.rotation
            }
            
            # Only log if position actually changed
            if not hasattr(self, 'last_logged_pose') or \
            abs(self.robot_pose['x'] - self.last_logged_pose.get('x', 0)) > 0.01 or \
            abs(self.robot_pose['y'] - self.last_logged_pose.get('y', 0)) > 0.01:
                
                self.get_logger().info(
                    f"Pose: ({self.robot_pose['x']:.2f}, {self.robot_pose['y']:.2f})"
                )
                self.last_logged_pose = self.robot_pose.copy()
                
        except TransformException as ex:
            self.get_logger().warning(
                f'TF lookup failed: {str(ex)[:100]}',
                throttle_duration_sec=2.0
            )
    
    def map_callback(self, msg):
        """Store SLAM map
        The map metadata (width, height, resolution, etc.) is stored in order for coordinate conversions (between map and world)
        """

        # Store map normally
        self.map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_resolution = msg.info.resolution
        self.map_origin = {
            'x': msg.info.origin.position.x,
            'y': msg.info.origin.position.y
        }
        self.map_width = msg.info.width
        self.map_height = msg.info.height

    def scan_callback(self, msg):
        """Process LiDAR"""
        self.laser_ranges = np.array(msg.ranges)
        # This replaces the inf and nan values with max range (which is what happens when no obstacle is detected in the given direction)
        self.laser_ranges = np.where(np.isfinite(self.laser_ranges), self.laser_ranges, msg.range_max)
        self.laser_msg = msg

    def world_to_map(self, x, y):
        """Convert world to map coordinates"""
        mx = int((x - self.map_origin['x']) / self.map_resolution)
        my = int((y - self.map_origin['y']) / self.map_resolution)
        return mx, my
    
    def map_to_world(self, mx, my):
        """Convert map to world coordinates"""
        x = mx * self.map_resolution + self.map_origin['x']
        y = my * self.map_resolution + self.map_origin['y']
        return x, y
    
    def is_free(self, mx, my, radius=0, allow_unknown=False):
        """Check if map cell is free
        0-49: Free space
        50-100: Occupied (obstacle)
        -1: Not explored yet (unknown)
        """
        for dx in range(-radius, radius+1):
            for dy in range(-radius, radius+1):
                cx, cy = mx + dx, my + dy
                if not (0 <= cx < self.map_width and 0 <= cy < self.map_height):
                    return False
                
                cell_value = self.map_data[cy, cx]
                
                # Treat unknown as free if allowed
                if allow_unknown and cell_value == -1:
                    continue
                
                if cell_value >= 50:  # obstacle
                    return False
        
        return True

    def astar_planning(self, start_x, start_y, goal_x, goal_y, allow_unknown=False):
        """A* path planning algorithm"""
        if self.map_data is None:
            self.get_logger().warning('No map available for planning')
            return None
        
        # Use smaller radius for exploration goals
        if allow_unknown:
            ROBOT_RADIUS = int(0.10 / self.map_resolution)  # Smaller: ~2 cells
            GOAL_RADIUS = 0  # Don't check radius at goal for frontiers
        else:
            ROBOT_RADIUS = int(0.25 / self.map_resolution)
            GOAL_RADIUS = ROBOT_RADIUS
        
        # Convert the start and goal locations to map coordinates
        start_mx, start_my = self.world_to_map(start_x, start_y)
        goal_mx, goal_my = self.world_to_map(goal_x, goal_y)

        # Check if the starting location is valid (if it is free)
        if not self.is_free(start_mx, start_my):
            self.get_logger().warning('Start position is not free')
            return None
        
        # Check if the goal location is valid (if it is free)
        if not self.is_free(goal_mx, goal_my, GOAL_RADIUS, allow_unknown=allow_unknown):
            self.get_logger().warning('Goal position is not free')
            # If the goal postiion is not free, it will find the nearest free cell
            goal_mx, goal_my = self.find_nearest_free(goal_mx, goal_my)
            if goal_mx is None:
                return None
            
        # A* algorithm
        open_set = []
        """Priority queue of nodes to explore, scored by f_score (estimated total cost)"""
        heapq.heappush(open_set, (0, (start_mx, start_my)))
        
        came_from = {}
        """Lists the Parent node for each visited node for path reconstruction"""
        g_score = {(start_mx, start_my): 0}
        """Actual cost from start to current node"""
        f_score = {(start_mx, start_my): self.heuristic(start_mx, start_my, goal_mx, goal_my)}
        """g_score + heuristic"""
        
        visited = set()
        

        while open_set:
            # Get the node with the lowest f_score
            _, current = heapq.heappop(open_set)
            
            # Skip the node if it has been visited already
            if current in visited:
                continue
            
            visited.add(current)
            self.publish_path(self.reconstruct_path(came_from, current), True)
            
            # If the goal is reached, reconstruct the path in order for the robot to actually be able to make it to the goal location
            if current == (goal_mx, goal_my):
                return self.reconstruct_path(came_from, current)
            
            # Check the neighbors of the current cell to see if they are free (8-connected - Robot can move horizontally, vertically, or diagonally)
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Path planning will skip neighbors with obstacles or neighbors that have been visited already
                if not self.is_free(neighbor[0], neighbor[1], radius=ROBOT_RADIUS, allow_unknown=allow_unknown):
                    continue
                
                if neighbor in visited:
                    continue
                
                move_cost = math.sqrt(dx**2 + dy**2)
                """
                Horizontal/Vertical cost: 1.0
                Diagonal cost: = sqrt{2} about 1.414
                """
                tentative_g = g_score[current] + move_cost
                
                # If a better path was found for the neighbor, update the scores and add it to the open_set
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(
                        neighbor[0], neighbor[1], goal_mx, goal_my
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # If open_set is empty without reaching the goal, it means that a path from the starting location to the goal location does not exist
        self.get_logger().warning('No path found!')
        return None
        
    
    def heuristic(self, x1, y1, x2, y2):
        """Euclidean distance heuristic
        Admissible heuristic - It will never overestimate the actual distance, which guarantees that A* finds the optimal path (if a path exists)
        """
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def reconstruct_path(self, came_from, current):
        """Reconstruct path from A* result"""

        # This follows the parent nodes backward from the goal to the start, and it then reverses the order
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        
        # Convert to world coordinates and simplify
        world_path = []
        for mx, my in path[::2]:  # Take every 5th waypoint in order to reduce path length
            wx, wy = self.map_to_world(mx, my)
            world_path.append((wx, wy))
        
        return world_path
    
    def find_nearest_free(self, mx, my, radius=20):
        """Find nearest free cell"""

        # The search takes place in expanding squares. The squear gets larger after all cells in the given square have been searche to determine if the cell is free
        for r in range(1, radius):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    if self.is_free(mx + dx, my + dy):
                        return mx + dx, my + dy
        return None, None
    
    def find_frontier(self):
        if self.map_data is None or self.robot_pose is None:
            return None

        robot_mx, robot_my = self.world_to_map(
            self.robot_pose['x'], self.robot_pose['y']
        )

        candidates = []
        MAX_RADIUS = int(15.0 / self.map_resolution)  # Increased search radius
        MIN_DIST = 0.8  # Minimum distance from robot

        for dy in range(-MAX_RADIUS, MAX_RADIUS):
            for dx in range(-MAX_RADIUS, MAX_RADIUS):
                mx = robot_mx + dx
                my = robot_my + dy

                # Check bounds
                if not (0 <= mx < self.map_width and 0 <= my < self.map_height):
                    continue

                # MUST be free space (0-49)
                cell_value = self.map_data[my, mx]
                if cell_value < 0 or cell_value >= 50:
                    continue

                # Check if adjacent to unknown space
                has_unknown_neighbor = False
                unknown_count = 0
                
                # Check 8-connected neighbors for more robust frontier detection
                for nx, ny in [(mx+1,my), (mx-1,my), (mx,my+1), (mx,my-1),
                               (mx+1,my+1), (mx+1,my-1), (mx-1,my+1), (mx-1,my-1)]:
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        if self.map_data[ny, nx] == -1:
                            has_unknown_neighbor = True
                            unknown_count += 1
                
                if not has_unknown_neighbor:
                    continue

                # Distance check
                dist = math.hypot(mx - robot_mx, my - robot_my)
                if dist * self.map_resolution < MIN_DIST:
                    continue
                
                wx, wy = self.map_to_world(mx, my)
                
                # Weight by number of unknown neighbors (prefer edges with more unknown space)
                candidates.append((dist, wx, wy, unknown_count))

        if not candidates:
            self.get_logger().warning(
                "No frontier found. Map may be fully explored.",
                throttle_duration_sec=2.0
            )
            return None

        self.get_logger().info(
            f"Found {len(candidates)} raw frontier cells",
            throttle_duration_sec=2.0
        )

        # Cluster frontiers with adjusted resolution
        clusters = {}
        CLUSTER_SIZE = 0.4  # Smaller clusters for better separation
        
        for dist, x, y, unk_count in candidates:
            key = (round(x / CLUSTER_SIZE) * CLUSTER_SIZE, 
                   round(y / CLUSTER_SIZE) * CLUSTER_SIZE)
            clusters.setdefault(key, []).append((dist, x, y, unk_count))

        # Filter out tiny clusters and compute cluster scores
        MIN_CLUSTER_SIZE = 3
        cluster_scores = []
        
        for cluster_points in clusters.values():
            if len(cluster_points) < MIN_CLUSTER_SIZE:
                continue
            
            # Centroid position
            avg_x = sum(p[1] for p in cluster_points) / len(cluster_points)
            avg_y = sum(p[2] for p in cluster_points) / len(cluster_points)
            
            # Distance from robot
            dist_to_centroid = math.hypot(
                avg_x - self.robot_pose['x'],
                avg_y - self.robot_pose['y']
            )
            
            # Average unknown neighbor count
            avg_unknown = sum(p[3] for p in cluster_points) / len(cluster_points)
            
            # Score: prefer larger clusters with more unknown space, but not too far
            # Score = size * unknown_factor / distance_factor
            size_score = len(cluster_points)
            unknown_score = avg_unknown
            distance_penalty = max(1.0, dist_to_centroid / 3.0)  # Penalty for distance
            
            score = (size_score * unknown_score) / distance_penalty
            
            cluster_scores.append((score, dist_to_centroid, avg_x, avg_y, len(cluster_points)))
        
        if not cluster_scores:
            self.get_logger().warning(
                "No valid frontier clusters found",
                throttle_duration_sec=2.0
            )
            return None
        
        # Sort by score (best first)
        cluster_scores.sort(reverse=True)
        
        # Return top 5 as list of (distance, x, y) for compatibility
        result = [(c[1], c[2], c[3]) for c in cluster_scores]
        
        best = cluster_scores[0]
        self.get_logger().info(
            f"Found {len(cluster_scores)} frontier clusters\n"
            f"  Best cluster: ({best[2]:.2f}, {best[3]:.2f})\n"
            f"    - Score: {best[0]:.2f}\n"
            f"    - Distance: {best[1]:.2f}m\n"
            f"    - Size: {best[4]} cells",
            throttle_duration_sec=2.0
        )
        
        return result
    
    def publish_path(self, path, test_paths=False):
        """Publish path for visualization
        RViz displays the path that is published
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "navigator_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        # ---- Appearance ----
        marker.scale.x = 0.05  # line width (meters)
        if not test_paths:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

        # ---- Convert poses to points ----
        marker.points = []

        if path:
            for x, y in path:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.05
                marker.points.append(p)
        
        self.path_pub.publish(marker)

    def follow_path(self):
        """Follow current A* path"""
        if not self.current_path or self.path_index >= len(self.current_path):
            return None
        
        # Gets the current waypoint the robot has to go to in order to continue on the planned path
        self.publish_path(self.current_path[max(0, self.path_index - 1):])
        waypoint = self.current_path[self.path_index]
        
        # Compute the distance between the robot and the waypoint
        dx = waypoint[0] - self.robot_pose['x']
        dy = waypoint[1] - self.robot_pose['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        # If the robot is close enough to the waypoint, the robot should move onto the next waypoint and start moving there.
        if distance < self.waypoint_tolerance:
            self.path_index += 1
            if self.path_index >= len(self.current_path):
                return None
            waypoint = self.current_path[self.path_index]
            dx = waypoint[0] - self.robot_pose['x']
            dy = waypoint[1] - self.robot_pose['y']
            distance = math.sqrt(dx**2 + dy**2)
        
        # Compute the angle to the waypoint
        target_angle = math.atan2(dy, dx)
        q = self.robot_pose['orientation']
        current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        angle_diff = target_angle - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        self.get_logger().info(
            f'Target: ({waypoint[0]:.2f}, {waypoint[1]:.2f}), '
            f'Current: ({self.robot_pose["x"]:.2f}, {self.robot_pose["y"]:.2f}), '
            f'Distance: {distance:.2f}m, Angle diff: {math.degrees(angle_diff):.1f}°',
            throttle_duration_sec=2.0
        )
        return distance, angle_diff

    def save_tag_database(self):
        """Save discovered tags"""
        data = {'discovered_tags': self.discovered_tags}
        try:
            with open(self.tag_database_path, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Save failed: {e}')
    
    def load_tag_database(self):
        """Load discovered tags"""
        if not os.path.exists(self.tag_database_path):
            return
        
        try:
            with open(self.tag_database_path, 'r') as f:
                data = json.load(f)
            self.discovered_tags = data.get('discovered_tags', {})
            # Convert string keys to int
            self.discovered_tags = {int(k): v for k, v in self.discovered_tags.items()}
            self.get_logger().info(f'Loaded {len(self.discovered_tags)} tags')
        except Exception as e:
            self.get_logger().error(f'Load failed: {e}')

    def find_best_frontier_toward_goal(self, goal_x, goal_y):
        """
        Find frontiers weighted by:
        - Distance from robot (closer is better)
        - Distance to goal (closer to goal is better)
        
        Returns list of frontiers sorted by combined score
        """
        if self.map_data is None or self.robot_pose is None:
            return None

        robot_mx, robot_my = self.world_to_map(
            self.robot_pose['x'], self.robot_pose['y']
        )

        candidates = []
        MAX_RADIUS = int(10.0 / self.map_resolution)

        for dy in range(-MAX_RADIUS, MAX_RADIUS):
            for dx in range(-MAX_RADIUS, MAX_RADIUS):
                mx = robot_mx + dx
                my = robot_my + dy

                if not (0 <= mx < self.map_width and 0 <= my < self.map_height):
                    continue

                if self.map_data[my, mx] < 0 or self.map_data[my, mx] >= 50:
                    continue

                # Check if this cell is adjacent to unknown space
                has_unknown_neighbor = False
                for nx, ny in [(mx+1,my), (mx-1,my), (mx,my+1), (mx,my-1)]:
                    if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                        if self.map_data[ny, nx] == -1:
                            has_unknown_neighbor = True
                            break
                
                if not has_unknown_neighbor:
                    continue

                min_clearance = float('inf')
                for check_r in range(1, 4):  # Check 3 cells around
                    for cdx in range(-check_r, check_r+1):
                        for cdy in range(-check_r, check_r+1):
                            cx, cy = mx + cdx, my + cdy
                            if 0 <= cx < self.map_width and 0 <= cy < self.map_height:
                                if self.map_data[cy, cx] >= 50:  # Obstacle
                                    dist = math.hypot(cdx, cdy)
                                    min_clearance = min(min_clearance, dist)
                
                # Skip frontiers too close to walls
                if min_clearance < 2.0:  # At least 2 cells clearance
                    continue

                wx, wy = self.map_to_world(mx, my)
                
                # Distance from robot
                dist_from_robot = math.hypot(wx - self.robot_pose['x'], wy - self.robot_pose['y'])
                
                if dist_from_robot < 0.6:  # Skip too close
                    continue
                
                # Distance to goal
                dist_to_goal = math.hypot(wx - goal_x, wy - goal_y)
                
                candidates.append((dist_from_robot, dist_to_goal, wx, wy))

        if not candidates:
            self.get_logger().warning(
                "No frontier found.",
                throttle_duration_sec=2.0
            )
            return None

        # Cluster frontiers
        clusters = {}
        for dist_robot, dist_goal, x, y in candidates:
            key = (round(x / 0.5) * 0.5, round(y / 0.5) * 0.5)
            clusters.setdefault(key, []).append((dist_robot, dist_goal, x, y))

        # Get cluster centroids with scores
        cluster_data = []
        for cluster_points in clusters.values():
            # Centroid position
            avg_x = sum(p[2] for p in cluster_points) / len(cluster_points)
            avg_y = sum(p[3] for p in cluster_points) / len(cluster_points)
            
            # Average distances
            avg_dist_robot = sum(p[0] for p in cluster_points) / len(cluster_points)
            avg_dist_goal = sum(p[1] for p in cluster_points) / len(cluster_points)
            
            cluster_data.append((avg_dist_robot, avg_dist_goal, avg_x, avg_y, len(cluster_points)))
        
        # Score function: weighted combination
        # Lower score is better
        ROBOT_WEIGHT = 0.3   # Weight for distance from robot (30%)
        GOAL_WEIGHT = 0.7    # Weight for distance to goal (70%)
        
        scored_frontiers = []
        for dist_robot, dist_goal, x, y, size in cluster_data:
            # Normalize distances to [0, 1] range
            max_dist_robot = max(c[0] for c in cluster_data)
            max_dist_goal = max(c[1] for c in cluster_data)
            
            norm_dist_robot = dist_robot / max_dist_robot if max_dist_robot > 0 else 0
            norm_dist_goal = dist_goal / max_dist_goal if max_dist_goal > 0 else 0
            
            # Combined score (lower is better)
            score = ROBOT_WEIGHT * norm_dist_robot + GOAL_WEIGHT * norm_dist_goal
            
            scored_frontiers.append((score, dist_robot, dist_goal, x, y))
        
        # Sort by score (best first)
        scored_frontiers.sort()
        
        # Return top 5 as list of (distance, x, y) for compatibility
        result = [(f[1], f[3], f[4]) for f in scored_frontiers[:5]]
        
        if result:
            best = scored_frontiers[0]
            self.get_logger().info(
                f"Found {len(candidates)} frontier candidates in {len(cluster_data)} clusters\n"
                f"  Best: ({best[3]:.2f}, {best[4]:.2f})\n"
                f"    - Distance from robot: {best[1]:.2f}m\n"
                f"    - Distance to goal: {best[2]:.2f}m\n"
                f"    - Score: {best[0]:.3f}",
                throttle_duration_sec=2.0
            )
        
        return result

    def quaternion_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
    
    def start_scan(self):
        q = self.robot_pose['orientation']
        self.scan_start_yaw = self.quaternion_to_yaw(q)
        self.last_yaw = self.scan_start_yaw
        self.total_yaw = 0.0

    def update_scan_rotation(self):
        q = self.robot_pose['orientation']
        current_yaw = self.quaternion_to_yaw(q)

        # Compute shortest signed angle difference
        delta = current_yaw - self.last_yaw
        delta = math.atan2(math.sin(delta), math.cos(delta))

        self.total_yaw += abs(delta)
        self.last_yaw = current_yaw

    def scan_complete(self):
        return self.total_yaw >= self.scan_target
    
    def compute_reachable_free_space(self):
        """Return a set of map cells reachable from robot through FREE space only"""
        start = self.world_to_map(self.robot_pose['x'], self.robot_pose['y'])
        visited = set()
        q = deque([start])

        while q:
            cx, cy = q.popleft()
            if (cx, cy) in visited:
                continue
            visited.add((cx, cy))

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                    if self.map_data[ny, nx] < 50 and self.map_data[ny, nx] != -1:
                        q.append((nx, ny))

        return visited
    
    def project_goal_to_reachable(self, goal_wx, goal_wy, reachable):
        gx, gy = self.world_to_map(goal_wx, goal_wy)

        best_cell = None
        best_dist = float('inf')

        for (mx, my) in reachable:
            if self.map_data[my, mx] >= 50:
                continue

            d = (mx - gx)**2 + (my - gy)**2
            if d < best_dist:
                best_dist = d
                best_cell = (mx, my)

        if best_cell is None:
            return None

        return self.map_to_world(*best_cell)
    
    def navigation_loop(self):
        """Main control loop"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        # Check for obstacles
        critical, warning, min_distance = self.check_obstacle_ahead()
        if self.target_tag_visible or self.target_tag_id in self.current_detections:
            if self.state != NavigationState.TRACKING:
                self.get_logger().info("Tag visible — skipping planning, entering TRACKING")
                self.state = NavigationState.TRACKING
                self.current_path = []
                self.path_index = 0
                self.cmd_vel_pub.publish(twist)
                return

        # Set the current speed to 0 when idle
        if self.state == NavigationState.IDLE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        elif self.state == NavigationState.PLANNING:
            if self.robot_pose is None:
                self.get_logger().info(f"Robot pose is NONE")
                return
            
            # ========== KNOWN TAG ==========
            if self.target_tag_id in self.discovered_tags:
                tag = self.discovered_tags[self.target_tag_id]
                
                # Vector from robot → tag
                dx = tag['x'] - self.robot_pose['x']
                dy = tag['y'] - self.robot_pose['y']
                dist = math.hypot(dx, dy)

                if dist > 0.001:
                    ux, uy = dx / dist, dy / dist
                else:
                    ux, uy = 0.0, 0.0

                # Stand-off goal in front of tag
                goal_x = tag['x'] - ux * self.approach_distance
                goal_y = tag['y'] - uy * self.approach_distance
                
                # Check if goal is in map bounds
                goal_mx, goal_my = self.world_to_map(goal_x, goal_y)
                goal_in_bounds = (0 <= goal_mx < self.map_width and 
                                0 <= goal_my < self.map_height)
                
                if goal_in_bounds and self.is_free(goal_mx, goal_my):
                    # Try direct path to tag
                    self.get_logger().info(f'Planning direct path to tag at ({goal_x:.2f}, {goal_y:.2f})')
                    self.current_path = self.astar_planning(
                        self.robot_pose['x'], self.robot_pose['y'],
                        goal_x, goal_y
                    )

                    if self.current_path:
                        self.path_index = 0
                        self.state = NavigationState.NAVIGATING
                        self.publish_path(self.current_path)
                        self.get_logger().info(f'Path planned to tag: {len(self.current_path)} waypoints')
                        # Exit planning - we're done!
                        self.cmd_vel_pub.publish(twist)
                        return
                    else:
                        self.get_logger().warning('Direct path to tag blocked')
                else:
                    self.get_logger().warning(
                        f'Unable to go directly to Tag goal ({goal_x:.2f}, {goal_y:.2f})'
                        f'using guided exploration'
                    )
                
                # If we reach here, either goal was out of bounds or direct path failed
                # Use goal-directed frontier exploration
                self.get_logger().info(f'Exploring toward tag at ({tag["x"]:.2f}, {tag["y"]:.2f})')
                frontiers = self.find_best_frontier_toward_goal(tag['x'], tag['y'])
                
                if not frontiers:
                    self.get_logger().warning('No frontiers toward goal, staying idle')
                    self.state = NavigationState.IDLE
                    self.cmd_vel_pub.publish(twist)
                    return
                
                # Try to plan to frontiers
                path_found = False
                for i, frontier in enumerate(frontiers):
                    self.get_logger().info(
                        f'Trying frontier {i+1}/{len(frontiers)}: ({frontier[1]:.2f}, {frontier[2]:.2f})'
                    )
                    
                    self.current_path = self.astar_planning(
                        self.robot_pose['x'], self.robot_pose['y'],
                        frontier[1], frontier[2], 
                        allow_unknown=True
                    )
                    
                    if self.current_path:
                        self.frontier_target = frontier
                        self.path_index = 0
                        self.state = NavigationState.NAVIGATING
                        self.publish_path(self.current_path)
                        self.get_logger().info(
                            f'Path planned to frontier {i+1}: {len(self.current_path)} waypoints'
                        )
                        path_found = True
                        break
                
                if not path_found:
                    self.get_logger().warning('No reachable frontier toward goal, staying idle')
                    self.state = NavigationState.IDLE
            
            # ========== UNKNOWN TAG ==========
            else:
                self.get_logger().info('Tag unknown, exploring frontiers')
                frontiers = self.find_frontier()
                
                if not frontiers:
                    self.get_logger().warning('No frontiers found, staying idle')
                    self.state = NavigationState.IDLE
                    self.cmd_vel_pub.publish(twist)
                    return
                
                path_found = False
                for i, frontier in enumerate(frontiers):
                    self.get_logger().info(
                        f'Trying frontier {i+1}/{len(frontiers)}: ({frontier[1]:.2f}, {frontier[2]:.2f})'
                    )
                    
                    self.current_path = self.astar_planning(
                        self.robot_pose['x'], self.robot_pose['y'],
                        frontier[1], frontier[2], 
                        allow_unknown=True
                    )
                    
                    if self.current_path:
                        self.frontier_target = frontier
                        self.path_index = 0
                        self.state = NavigationState.NAVIGATING
                        self.publish_path(self.current_path)
                        self.get_logger().info(
                            f'Path planned to frontier {i+1}: {len(self.current_path)} waypoints'
                        )
                        path_found = True
                        break
                
                if not path_found:
                    self.get_logger().warning('No reachable frontier found, staying idle')
                    self.state = NavigationState.IDLE

        elif self.state == NavigationState.NAVIGATING:
            # If the target tag is visible, then the robot should start TRACKING the tag and move directly to it.
            if self.target_tag_visible:
                self.state = NavigationState.TRACKING
                self.current_path = []
                self.path_index = 0
                self.get_logger().info('Switching to visual tracking')
                self.cmd_vel_pub.publish(twist)
                return
            # If the target tag is not visible, the robot should continue to follow the planned path
            nav_info = self.follow_path()
            # the robot completed the path
            if nav_info is None:
                # If the target AprilTag has been discovered, the robot should then move directly to the tag
                if self.target_tag_id in self.discovered_tags:
                    if self.target_tag_id in self.current_detections:
                        self.state = NavigationState.TRACKING
                    else:
                        self.state = NavigationState.PLANNING
                else:
                    self.start_scan()
                    self.state = NavigationState.SCANNING
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return
            
            # Robot should determine the distance and angle to the next waypoint and move towards it.
            distance, angle_diff = nav_info
            
            if critical:
                # Emergency stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warning(
                    f'EMERGENCY STOP! Obstacle at {min_distance:.2f}m',
                    throttle_duration_sec=1.0
                )
                self.current_path = []
                self.path_index = 0
                self.frontier_target = None
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = NavigationState.RECOVERY
                self.recovery_start_time = self.get_clock().now()
                self.recovery_phase = 0
                self.cmd_vel_pub.publish(twist)
                return
            elif warning:
                # Slow down near obstacles
                speed_factor = (min_distance - self.critical_distance) / \
                                (self.min_wall_distance - self.critical_distance)
                speed_factor = max(0.2, min(1.0, speed_factor))
                
                if abs(angle_diff) > 0.5:
                    twist.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
                    twist.linear.x = 0.0 
                else:
                    twist.linear.x = self.linear_speed * speed_factor
                    twist.angular.z = self.angular_speed * angle_diff * 2.0
            else:
                # Normal navigation
                if abs(angle_diff) > 0.5:
                    twist.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
                    twist.linear.x = 0.0 
                elif abs(angle_diff) > 0.2:
                    twist.angular.z = self.angular_speed * (angle_diff / abs(angle_diff))
                    twist.linear.x = self.linear_speed * 0.5
                else:
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed * angle_diff * 2.0
            
            self.get_logger().info(
                f'Nav: wp {self.path_index}/{len(self.current_path)}, '
                f'dist={distance:.2f}m, angle={math.degrees(angle_diff):.1f}°',
                throttle_duration_sec=1.0
            )
        
        elif self.state == NavigationState.TRACKING:
            # If the tag is no longer visible, replan the path to the tag

            tag = self.discovered_tags[self.target_tag_id]
            tag_x = tag['x']
            tag_y = tag['y']

            dx = tag_x - self.robot_pose['x']
            dy = tag_y - self.robot_pose['y']
            dist = math.hypot(dx, dy)

            ux, uy = dx / dist, dy / dist

            raw_goal_x = tag_x - ux * self.approach_distance
            raw_goal_y = tag_y - uy * self.approach_distance

            reachable = self.compute_reachable_free_space()

            goal_x, goal_y = self.project_goal_to_reachable(
                raw_goal_x, raw_goal_y, reachable
            )
            if self.tracked_goal is None:
                self.tracked_goal = (goal_x, goal_y)
            else:
                goal_moved_distance = math.hypot(
                    goal_x - self.tracked_goal[0],
                    goal_y - self.tracked_goal[1]
                )
                REPLAN_THRESHOLD = 0.5  # Replan if goal moves more than 0.5m
    
                if goal_moved_distance > REPLAN_THRESHOLD:
                    self.get_logger().info(
                        f'Tag moved {goal_moved_distance:.2f}m - replanning path'
                    )
                    self.current_path = []
                    self.path_index = 0
                    self.tracked_goal = (goal_x, goal_y)

            if not self.current_path and dist > self.approach_distance:
                self.current_path = self.astar_planning(
                    self.robot_pose['x'], self.robot_pose['y'],
                    goal_x, goal_y,
                    allow_unknown=False
                )

                if self.current_path:
                    self.path_index = 0
                    self.publish_path(self.current_path)
                    self.get_logger().info(f'Path planned to tag: {len(self.current_path)} waypoints')
            
            nav_info = self.follow_path()
            if nav_info is None:
                self.state = NavigationState.REACHED
                self.get_logger().info(f'REACHED tag {self.target_tag_id}!')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.target_tag_visible = False
                self.target_tag_id = 11
                self.tracked_goal = None
                return
            
            distance, angle_diff = nav_info
            
            if critical:
                # Emergency stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warning(
                    f'EMERGENCY STOP! Obstacle at {min_distance:.2f}m',
                    throttle_duration_sec=1.0
                )
                self.current_path = []
                self.path_index = 0
                self.frontier_target = None 
                self.state = NavigationState.RECOVERY
                self.recovery_phase = 0
                self.recovery_start_time = self.get_clock().now()
            elif warning:
                # Slow down near obstacles
                speed_factor = (min_distance - self.critical_distance) / \
                                (self.min_wall_distance - self.critical_distance)
                speed_factor = max(0.2, min(1.0, speed_factor))
                
                if abs(angle_diff) > 0.5:
                    twist.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
                    twist.linear.x = 0.0 
                else:
                    twist.linear.x = self.linear_speed * speed_factor
                    twist.angular.z = self.angular_speed * angle_diff * 2.0
            else:
                # Normal navigation
                if abs(angle_diff) > 0.5:
                    twist.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
                    twist.linear.x = 0.0 
                elif abs(angle_diff) > 0.2:
                    twist.angular.z = self.angular_speed * (angle_diff / abs(angle_diff))
                    twist.linear.x = self.linear_speed * 0.5
                else:
                    twist.linear.x = self.linear_speed
                    twist.angular.z = self.angular_speed * angle_diff * 2.0
            
            self.get_logger().info(
                f'Nav: wp {self.path_index}/{len(self.current_path)}, '
                f'dist={distance:.2f}m, angle={math.degrees(angle_diff):.1f}°',
                throttle_duration_sec=1.0
            )
        # If the robot has reached the tag, the robot should become idle and wait for new commands
        elif self.state == NavigationState.REACHED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.state = NavigationState.IDLE
            msg = Bool()
            msg.data = True
            self.reach_goal_pub.publish(msg)
            self.target_tag_visible = False
            self.target_tag_id = 11
            self.current_path = []
            self.tracked_goal = None
            self.publish_path(self.current_path)
            self.get_logger().info('Mission complete!', throttle_duration_sec=3.0)
        elif self.state == NavigationState.RECOVERY:
            now = self.get_clock().now()
            elapsed = (now - self.recovery_start_time).nanoseconds * 1e-9

            # Phase 0: back up
            if self.recovery_phase == 0:
                twist.linear.x = -self.recover_back_speed
                twist.angular.z = 0.0

                if elapsed > self.recovery_back_time:
                    self.recovery_phase = 1
                    self.recovery_start_time = now

            # Phase 1: turn away
            elif self.recovery_phase == 1:
                if self.laser_ranges is not None:
                    left = np.mean(self.laser_ranges[len(self.laser_ranges)//2:])
                    right = np.mean(self.laser_ranges[:len(self.laser_ranges)//2])
                    twist.angular.z = self.recovery_turn_speed if left > right else -self.recovery_turn_speed

                if elapsed > 0.8:
                    self.recovery_phase = 2

            # Phase 2: done → replan
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = NavigationState.PLANNING
                self.get_logger().info('Recovery complete — replanning')

            self.cmd_vel_pub.publish(twist)
            return
        elif self.state == NavigationState.SCANNING:
            self.update_scan_rotation()

            twist.linear.x = 0.0
            twist.angular.z = self.scan_speed

            if self.scan_complete():
                self.get_logger().info("Full 360 scan complete")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = NavigationState.PLANNING

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    navigator = AprilTagNavigator()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(navigator)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.save_tag_database()
        twist = Twist()
        navigator.cmd_vel_pub.publish(twist)
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()