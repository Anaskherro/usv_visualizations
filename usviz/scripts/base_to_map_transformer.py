import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from math import sin, cos, pi
import tf2_ros
import tf_transformations


class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('base_link_to_map_tf_broadcaster')

        # Timer to broadcast the transform at a regular interval
        self.timer = self.create_timer(0.001, self.broadcast_transform)  # 10 Hz

        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create an Odometry subscriber to get position and orientation
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Simulated variables for the USV's position and orientation
        self.x = 0.0  # Position x (meters)
        self.y = 0.0  # Position y (meters)
        self.yaw = 0.0  # Orientation in radians

    def odom_callback(self, msg):
        # Extract position and orientation from the Odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract the orientation quaternion from the Odometry message
        quat = msg.pose.pose.orientation
        _, _, self.yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

    def broadcast_transform(self):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the timestamp and frame IDs
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'  # Global frame
        t.child_frame_id = 'base_link'  # USV frame

        # Set the translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # No altitude for this example

        # Set the rotation (convert yaw to a quaternion)
        qx, qy, qz, qw = self.yaw_to_quaternion(self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert a yaw angle (in radians) to a quaternion."""
        qx = 0.0
        qy = 0.0
        qz = sin(yaw / 2.0)
        qw = cos(yaw / 2.0)
        return qx, qy, qz, qw

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert a quaternion to euler angles (roll, pitch, yaw)."""
        euler = tf_transformations.euler_from_quaternion([x, y, z, w])
        return euler  # returns (roll, pitch, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = TransformBroadcasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

