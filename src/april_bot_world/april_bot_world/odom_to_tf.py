import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # REQUIRED: use sim time
        self.declare_parameter('use_sim_time', True)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.latest_odom = None

        # Subscribe to odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publish TF at 20 Hz (safe rate)
        self.timer = self.create_timer(0.05, self.publish_tf)

        self.get_logger().info('Odom to TF converter started (20 Hz, sim time)')

    def odom_callback(self, msg):
        self.latest_odom = msg

    def publish_tf(self):
        if self.latest_odom is None:
            return

        msg = self.latest_odom

        t = TransformStamped()

        # CRITICAL: use current sim time
        t.header.stamp = self.get_clock().now().to_msg()

        # MUST be these frames
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0  # IMPORTANT: footprint stays on ground

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
