import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Int32, Bool
from tf2_ros import Buffer, TransformListener, TransformException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import numpy as np
import math
from enum import Enum
import json
import os
import heapq

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

class AprilTagNavigator(Node):
    """Uses A* algorithm for path planning to get the robot to move to the given goal id. If it knows where the tag is, it will go directly there, otherwise it will begin searching for the april tag in unexplored locations"""
    def __init__(self):
        super().__init__('navigator')

        # Parameters
        self.declare_parameter('approach_distance', 0.5)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('waypoint_tolerance', 0.3)
        self.declare_parameter('tag_database_path', 'discovered_tags.json')
        self.declare_parameter('frontier_grid_size', 0.5)
        self.declare_parameter('cancel_command', 11)
        
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

        # TF2 for SLAM localization
        try:
            use_sim = self.get_parameter('use_sim_time').value
            self.get_logger().info(f'use_sim_time: {use_sim}')
        except:
            self.get_logger().warn('use_sim_time not set!')
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
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
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        """Publishes A* path for visualization in RViz. Used for debugging path."""
        self.reach_goal_pub = self.create_publisher(Bool, '/reach_goal', 10)
        """Publishes boolean if the robot has reached the given goal"""


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

        # Timers
        self.map_wait_timer = self.create_timer(1.0, self.wait_for_map_timer)
        """Wait for /map to begin publishing"""

        self.pose_timer = None
        """Timer to update the robot pose every 0.2 secconds"""

        self.nav_timer = None
        """Timer to run the navigation loop to move the robot"""
        
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
                rclpy.time.Time(),
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
                rclpy.time.Time(),  # Get latest
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
            0.1,
            self.navigation_loop
        )

        # Ensure robot starts stopped
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        # Initialize state machine
        self.state = NavigationState.IDLE

    def goal_callback(self, msg):
        """Receive target tag ID"""
        self.target_tag_id = msg.data
        self.state = NavigationState.PLANNING
        # If cancel command is given, the robot should become idle.
        if self.target_tag_id == self.cancel_command:
            self.state = NavigationState.IDLE
        elif self.target_tag_id in self.discovered_tags:
            self.get_logger().info(f'Planning path to KNOWN tag {self.target_tag_id}')
        else:
            self.get_logger().info(f'Searching for UNKNOWN tag {self.target_tag_id}')

    def detection_callback(self, msg):
        """Receive AprilTags detection"""

        # Need to clear out all perviously seen tags before listing the newly seen ones
        self.current_detections = {}
        self.target_tag_visible = False
        for detection in msg.detections:
            # Get tag_id and then add it to the current_detections with its pose
            tag_id = detection.id
            tag_frame = f"apriltag_{tag_id}"
            self.current_detections[tag_id] = {
                'pose': detection.pose
            }
            try:
                # Use tf in order to fine the [x,y,z] coordinates for the seen AprilTag
                transform = self.tf_buffer.lookup_transform(
                'map', tag_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Store in database
                self.discovered_tags[tag_id] = {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                }
                
                # Print the [x,y] location of the AprilTag
                self.get_logger().info(
                    f'Tag {tag_id} at ({transform.transform.translation.x:.2f}, '
                    f'{transform.transform.translation.y:.2f})',
                    throttle_duration_sec=2.0
                )

            except TransformException:
                pass
            # If the seen tag is the target AprilTag, mark it as seen and calculate the distance and angle to the target tag in order to update the state to TRACKING
            if tag_id == self.target_tag_id:
                self.target_tag_visible = True
                pose = detection.pose
                self.target_tag_distance = math.sqrt(
                    pose.position.x**2 + pose.position.y**2 + pose.position.z**2
                )
                self.target_tag_angle = math.degrees(math.atan2(pose.position.y, pose.position.x))

    def update_robot_pose(self):
        """Get robot pose from SLAM"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.robot_pose = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'orientation': transform.transform.rotation
            }
            self.get_logger().info(
                f"Pose: ({self.robot_pose['x']:.2f}, {self.robot_pose['y']:.2f})",
                throttle_duration_sec=2.0
            )
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform map to base_footprint: {ex}',
                throttle_duration_sec=5.0
            )
            try:
                # Check if odom->base_footprint exists (should be from Gazebo)
                transform = self.tf_buffer.lookup_transform(
                    'odom', 'base_footprint',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                self.robot_pose = {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'orientation': transform.transform.rotation
                }
                self.get_logger().info(
                    'Using odom frame (SLAM not ready yet)',
                    throttle_duration_sec=5.0
                )
                
            except TransformException as ex:
                self.get_logger().warn(
                    f'Could not get robot pose: {ex}',
                    throttle_duration_sec=5.0
                )
    
    def map_callback(self, msg):
        """Store SLAM map
        The map metadata (width, height, resolution, etc.) is stored in order for coordinate conversions (between map and world)
        """
        # Convert the flat array to 2D grid. 
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
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
    
    def is_free(self, mx, my, radius=0):
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
                if self.map_data[cy, cx] >= 50:   # obstacle
                    return False
        return True

    def astar_planning(self, start_x, start_y, goal_x, goal_y):
        """A* path planning algorithm"""
        if self.map_data is None:
            self.get_logger().warn('No map available for planning')
            return None
        
        # Convert the start and goal locations to map coordinates
        start_mx, start_my = self.world_to_map(start_x, start_y)
        goal_mx, goal_my = self.world_to_map(goal_x, goal_y)

        # Check if the starting location is valid (if it is free)
        if not self.is_free(start_mx, start_my):
            self.get_logger().warn('Start position is not free')
            return None
        
        # Check if the goal location is valid (if it is free)
        if not self.is_free(goal_mx, goal_my):
            self.get_logger().warn('Goal position is not free')
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

            # If the goal is reached, reconstruct the path in order for the robot to actually be able to make it to the goal location
            if current == (goal_mx, goal_my):
                return self.reconstruct_path(came_from, current)
            
            # Check the neighbors of the current cell to see if they are free (8-connected - Robot can move horizontally, vertically, or diagonally)
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Path planning will skip neighbors with obstacles or neighbors that have been visited already
                if not self.is_free(neighbor[0], neighbor[1]):
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
        self.get_logger().warn('No path found!')
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
        for mx, my in path[::5]:  # Take every 5th waypoint in order to reduce path length
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

        for my in range(1, self.map_height - 1):
            for mx in range(1, self.map_width - 1):

                # must be free
                if self.map_data[my, mx] != 0:
                    continue

                # check if adjacent to unknown
                neighbors = [
                    self.map_data[my+1, mx],
                    self.map_data[my-1, mx],
                    self.map_data[my, mx+1],
                    self.map_data[my, mx-1],
                ]

                if -1 not in neighbors:
                    continue

                # distance to robot
                dist = math.hypot(mx - robot_mx, my - robot_my)
                wx, wy = self.map_to_world(mx, my)
                candidates.append((dist, wx, wy))

        if not candidates:
            self.get_logger().warn("No frontier found")
            return None

        candidates.sort()
        return {'x': candidates[0][1], 'y': candidates[0][2]}
    
    def publish_path(self, path):
        """Publish path for visualization
        RViz displays the path that is published
        """
        if not path:
            return
        
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)

    def follow_path(self):
        """Follow current A* path"""
        if not self.current_path or self.path_index >= len(self.current_path):
            return None
        
        # Gets the current waypoint the robot has to go to in order to continue on the planned path
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
        
        # Compute the angle to the waypoint
        target_angle = math.atan2(dy, dx)
        q = self.robot_pose['orientation']
        current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        angle_diff = target_angle - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
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
    
    def navigation_loop(self):
        """Main control loop"""
        twist = Twist()

        # Set the current speed to 0 when idle
        if self.state == NavigationState.IDLE:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        elif self.state == NavigationState.PLANNING:
            if self.robot_pose is None:
                self.get_logger().info(f"Robot pose is NONE")
                return
            
            # The tag is known, so the PLANNING state will run A* from the current position to the known tag location
            if self.target_tag_id in self.discovered_tags:
                # Plan to known tag
                tag = self.discovered_tags[self.target_tag_id]
                self.current_path = self.astar_planning(
                    self.robot_pose['x'], self.robot_pose['y'],
                    tag['x'], tag['y']
                )

                if self.current_path:
                    self.path_index = 0
                    self.state = NavigationState.NAVIGATING
                    self.publish_path(self.current_path)
                    self.get_logger().info(f'Path planned: {len(self.current_path)} waypoints')
                else:
                    self.state = NavigationState.IDLE
            else:
                # The tag is unknown, so the PLANNING state will run A* from the current position to the closes unexplored location
                self.frontier_target = self.find_frontier()
                if self.frontier_target:
                    self.current_path = self.astar_planning(
                        self.robot_pose['x'], self.robot_pose['y'],
                        self.frontier_target['x'], self.frontier_target['y']
                    )
                    
                    if self.current_path:
                        self.path_index = 0
                        self.state = NavigationState.NAVIGATING
                        self.publish_path(self.current_path)
                        self.get_logger().info('Exploring frontier')
                else:
                    self.get_logger().warn('No frontier found')
                    self.state = NavigationState.PLANNING

        elif self.state == NavigationState.NAVIGATING:
            # If the target tag is visible, then the robot should start TRACKING the tag and move directly to it.
            if self.target_tag_visible:
                self.state = NavigationState.TRACKING
                self.get_logger().info('Switching to visual tracking')
                return
            # If the target tag is not visible, the robot should continue to follow the planned path
            nav_info = self.follow_path()
            # the robot completed the path
            if nav_info is None:
                # If the target AprilTag has been discovered, the robot should then move directly to the tag
                if self.target_tag_id in self.discovered_tags:
                    self.state = NavigationState.TRACKING
                else:
                    # If the target AprilTag has not been discovered, the robot should continue PLANNING and move onto the next closest unexplored location
                    self.state = NavigationState.PLANNING
                return
            
            # Robot should determine the distance and angle to the next waypoint and move towards it.
            distance, angle_diff = nav_info
            
            if abs(angle_diff) > 0.4:
                twist.angular.z = self.angular_speed * (1.0 if angle_diff > 0 else -1.0)
                twist.linear.x = self.linear_speed * 0.5
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = self.angular_speed * angle_diff
            
            self.get_logger().info(
                f'Following path: waypoint {self.path_index}/{len(self.current_path)}',
                throttle_duration_sec=1.0
            )
        
        elif self.state == NavigationState.TRACKING:
            # If the tag is no longer visible, replan the path to the tag
            if not self.target_tag_visible:
                self.state = NavigationState.PLANNING
                self.get_logger().warn('Lost tag, replanning')
                return
            
            # If the robot has reached the april tag, change the state to REACHED and publish that the tag was reached
            if self.target_tag_distance <= self.approach_distance:
                self.state = NavigationState.REACHED
                self.get_logger().info(f'REACHED tag {self.target_tag_id}!')
            # If the robot has not been reached and is still visible, move toward the tag
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = -self.angular_speed * (self.target_tag_angle / 30.0)
                
                self.get_logger().info(
                    f'Tracking: {self.target_tag_distance:.2f}m',
                    throttle_duration_sec=1.0
                )
        # If the robot has reached the tag, the robot should become idle and wait for new commands
        elif self.state == NavigationState.REACHED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.state = NavigationState.IDLE
            self.reach_goal_pub.publish(True)
            self.get_logger().info('Mission complete!', throttle_duration_sec=3.0)
        
        # Publish the velocity of the robot
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    navigator = AprilTagNavigator()
    try:
        rclpy.spin(navigator)
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