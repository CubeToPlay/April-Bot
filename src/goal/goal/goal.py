import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool

class Goal(Node):
    def __init__(self):
        super().__init__('goal')
    
        self.subscriber = self.create_subscription(Int32, '/gesture', self.gesture_callback, 1)
        self.goal_subscriber = self.create_subscription(Bool, '/reach_goal', self.goal_reached_callback, 1)
        self.goal_publisher = self.create_publisher(Int32, '/goal_id', 10)

        self.initial_time = 0
        self.timer_started = False
        self.goal = -1 # Default to no gesture detected

    def gesture_callback(self, msg):

        # Checks if timer is started and if the gesture has been up for more than 3 seconds, publishes goal if so
        if ((time.time() - self.initial_time > 2) and (self.timer_started == True) and (msg.data > 0)):
            self.goal_publisher.publish(msg)
            self.timer_started = False

        # Checks if the current goal is the same as the previous goal and if timer is not started yet, starts timer if so
        if ((self.compare_goal(msg) == True) and (self.timer_started == False)):
            self.initial_time = time.time()
            self.timer_started = True
        
        # Checks if the current goal is not the same as the previous goal, stops timer if so
        if ((self.compare_goal(msg) == False) or (msg.data < 0)) :
            self.timer_started = False

        self.goal = msg
        
    # Compares the current goal to the previous goal and returns True or False
    def compare_goal(self, msg):
        if (self.goal == msg):
            return True
        return False

    # Publishes cancel command if the goal has been reached
    def goal_reached_callback(self, msg):
        if (msg.data):
            self.goal_publisher.publish(11)

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