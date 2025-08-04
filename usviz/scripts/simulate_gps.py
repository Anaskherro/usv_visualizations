import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        # Create a publisher for the /fix topic
        self.gps_pub = self.create_publisher(NavSatFix, '/fix', 10)

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

        # Interval in seconds between messages
        self.interval = 1.0

        # Start publishing GPS data
        self.index = 0
        self.timer = self.create_timer(self.interval, self.publish_next_gps)

    def publish_next_gps(self):
        """Publish the next GPS coordinate with a timer."""
        # Get the current GPS coordinate
        lat, lon = self.gps_data[self.index]

        # Create a NavSatFix message
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = 'gps'  # Frame ID
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = 0.0  # Assuming 0 altitude for now

        # Publish the message
        self.gps_pub.publish(gps_msg)
        self.get_logger().info(f'Published GPS coordinate: lat={lat}, lon={lon}')

        # Move to the next coordinate (loop back to the first point if at the end)
        self.index = (self.index + 1) % len(self.gps_data)


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

