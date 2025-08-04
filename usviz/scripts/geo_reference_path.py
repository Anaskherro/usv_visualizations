import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from pyproj import Proj, transform
from nav_msgs.msg import Path


class GPSToPathPublisher(Node):
    def __init__(self):
        super().__init__('gps_to_path_publisher')

        # Create a Path publisher
        self.path_pub = self.create_publisher(Path, '/gps_path', 10)

        # Define WGS84 (GPS) and UTM projection
        self.wgs84 = Proj(proj="latlong", datum="WGS84")
        self.utm_proj = Proj(proj="utm", zone=29, datum="WGS84")  # Zone 29

        # Reference GPS point
        self.reference_lat = 32.211375  # Lat Vanguard Center
        self.reference_lon = -7.938493  # Lon Vanguard Center

        # Predefined GPS data (latitude, longitude)
        self.gps_data = [
            (32.2115313, -7.9384192),
            (32.2115177, -7.9383991),
            (32.2115109, -7.9383830),
            (32.2115064, -7.9383508),
            (32.2115279, -7.9383092),
            (32.2115654, -7.9382998),
            (32.2115824, -7.9383240),
            (32.2115858, -7.9383575),
            (32.2115801, -7.9383790),
            (32.2115756, -7.9383924),
            (32.2115665, -7.9384018),
            (32.2115563, -7.9384098)
        ]

        # Create a timer to publish the path at a fixed rate
        self.timer = self.create_timer(0.001, self.publish_path)  # Publish every 1 second

        self.get_logger().info('Node initialized. Publishing GPS path at 1 Hz.')

    def gps_to_xy(self, lat, lon):
        """Convert GPS coordinates to local x, y coordinates."""
        ref_x, ref_y = transform(self.wgs84, self.utm_proj, self.reference_lon, self.reference_lat)
        x, y = transform(self.wgs84, self.utm_proj, lon, lat)
        return x - ref_x, y - ref_y

    def create_path(self, gps_data):
        """Create a Path message from GPS data."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'  # Coordinate frame (e.g., 'map' or 'odom')

        for lat, lon in gps_data:
            x, y = self.gps_to_xy(lat, lon)
            pose = PoseStamped()
            pose.header.stamp = path.header.stamp
            pose.header.frame_id = path.header.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # Assuming 2D path, no altitude information
            path.poses.append(pose)

        return path

    def publish_path(self):
        """Publish the GPS path."""
        path = self.create_path(self.gps_data)
        self.path_pub.publish(path)
        self.get_logger().info('Publishing GPS path')


def main(args=None):
    rclpy.init(args=args)
    node = GPSToPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

