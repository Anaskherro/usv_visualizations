import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomPathPublisher(Node):
    def __init__(self):
        super().__init__('odom_path_publisher')

        # Create a publisher for the /odom_path topic
        self.path_pub = self.create_publisher(Path, '/odom_path', 10)

        # Subscribe to the /odom topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Initialize the Path message
        self.path = Path()
        self.path.header.frame_id = 'map'  # Set the frame_id for the path

    def odom_callback(self, msg: Odometry):
        """Callback function for /odom messages."""
        # Create a PoseStamped from the received Odometry message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Append the pose to the path
        self.path.poses.append(pose)

        # Update the header stamp for the Path message
        self.path.header.stamp = self.get_clock().now().to_msg()

        # Publish the path
        self.path_pub.publish(self.path)
        self.get_logger().info('Published updated odometry path')


def main(args=None):
    rclpy.init(args=args)
    node = OdomPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
