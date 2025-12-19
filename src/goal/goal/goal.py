import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int32, Bool

class Goal(Node):
    def __init__(self):
        super().__init__('goal')
    
        # The gestures evaluated
        self.subscriber = self.create_subscription(Int32, '/gesture', self.gesture_callback, 1)
        
        # If the goal has been reached
        self.goal_subscriber = self.create_subscription(Bool, '/reach_goal', self.goal_reached_callback, 1)
        
        self.goal_publisher = self.create_publisher(Int32, '/goal_id', 10)

        self.gesture_time = 2 # Sets how long the gesture needs to be held up (in seconds)
        self.initial_time = 0
        self.timer_started = False
        
        self.waiting_for_completion = False # Used to wait for a response from the path-planning
        
        self.goal = -1 # Default to no gesture detected

    # Uses a timer to time how long a gesture has been held up. If the gesture has been held up for longer than gesture_time
    # then that gesture's /goal_id will be published
    def gesture_callback(self, msg : Int32):
        gesture = msg.data
        
        # Checks if timer is started and if the gesture has been up for more than 3 seconds, publishes goal if so
        if (((time.time() - self.initial_time) > self.gesture_time) and self.timer_started):
            self.get_logger().info(f"Gesture {gesture} chosen")
            if (not self.waiting_for_completion) or (gesture == 11):
                self.get_logger().info(f"Publishing gesture {gesture}")
                self.goal_publisher.publish(msg)
                self.waiting_for_completion = (gesture != 11)
            else:
                self.get_logger().info("Cannot publish gesture. Waiting until goal reached")
            
            self.timer_started = False
        
        # If the current goal is the same as the previous goal and if timer is not started yet, start timer
        if ((self.goal == gesture) and not self.timer_started):
            self.initial_time = time.time()
            self.timer_started = True
        
        # If the current goal is not the same as the previous goal, stop timer
        if ((self.goal != gesture) or (gesture < 1)) :
            self.timer_started = False
        
        self.goal = gesture

    # Publishes cancel command if the goal has been reached (True received from msg)
    def goal_reached_callback(self, msg : Bool):        
        if msg.data:
            self.waiting_for_completion = False

    # Deletes node
    def cleanup(self):
        self.get_logger().info("Goal Node Shutting Down")
        self.destroy_node()

# Spins up node from main entry point, deletes node on keyboard interrupt
def main(args=None):
    rclpy.init(args=args)
    goal = Goal()
    try:
        rclpy.spin(goal)
    except KeyboardInterrupt:
        pass
    
    goal.cleanup()
    
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
        main()