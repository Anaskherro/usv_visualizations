import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from pyproj import Proj, transform
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

# Global variables to store sensor data
current_lat = None
current_lon = None
current_orientation = None

# Define WGS84 (GPS) and a UTM projection
wgs84 = Proj(proj="latlong", datum="WGS84")
utm_proj = Proj(proj="utm", zone=29, datum="WGS84")  # Replace zone as needed

# Reference GPS point for the local frame
reference_lat = 32.211375  # Lat Vanguard Center
reference_lon = -7.938493  # Lon Vanguard Center

def gps_to_xy(lat, lon):
    """Convert GPS coordinates to local x, y coordinates."""
    global reference_lat, reference_lon
    if reference_lat is None or reference_lon is None:
        reference_lat = lat
        reference_lon = lon

    ref_x, ref_y = transform(wgs84, utm_proj, reference_lon, reference_lat)
    x, y = transform(wgs84, utm_proj, lon, lat)
    return x - ref_x, y - ref_y

def quaternion_to_yaw(quaternion):
    """Convert a quaternion to a yaw angle (heading)."""
    _, _, yaw = euler_from_quaternion([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    ])
    return yaw

def create_odometry(x, y, yaw):
    """Create an Odometry message from x, y, and yaw."""
    odometry = Odometry()
    
    # Set the header
    odometry.header.stamp = rclpy.time.Time().to_msg()  # Use ROS time
    odometry.header.frame_id = 'map'  # Replace with the parent frame
    odometry.child_frame_id = 'base_link'  # Replace with the USV's frame

    # Set the pose
    odometry.pose.pose.position.x = x
    odometry.pose.pose.position.y = y
    odometry.pose.pose.position.z = 0.0  # No altitude
    odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, \
    odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w = quaternion_from_euler(0, 0, yaw)

    # Optionally set covariance (if required)
    odometry.pose.covariance = [0.0] * 36  # 6x6 covariance matrix (diagonal or specific values)

    # Velocity fields (set to 0 since no velocity info is available in this example)
    odometry.twist.twist.linear.x = 0.0
    odometry.twist.twist.linear.y = 0.0
    odometry.twist.twist.linear.z = 0.0
    odometry.twist.twist.angular.x = 0.0
    odometry.twist.twist.angular.y = 0.0
    odometry.twist.twist.angular.z = 0.0

    return odometry

class USVOdometryPublisher(Node):
    def __init__(self):
        super().__init__('usv_odometry_publisher')

        # Subscribers
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.timer = self.create_timer(0.001, self.timer_callback)  # 10 Hz

    def gps_callback(self, msg):
        global current_lat, current_lon
        current_lat = msg.latitude
        current_lon = msg.longitude

    def imu_callback(self, msg):
        global current_orientation
        current_orientation = msg.orientation

    def timer_callback(self):
        if current_lat is not None and current_lon is not None and current_orientation is not None:
            # Convert GPS to local frame
            x, y = gps_to_xy(current_lat, current_lon)

            # Get yaw from IMU orientation
            yaw = quaternion_to_yaw(current_orientation)

            # Create and publish odometry
            odometry = create_odometry(x, y, yaw)
            self.odom_pub.publish(odometry)

def main(args=None):
    rclpy.init(args=args)
    node = USVOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

