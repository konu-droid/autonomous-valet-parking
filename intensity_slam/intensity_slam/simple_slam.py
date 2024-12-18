import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros import LookupException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf_transformations import quaternion_matrix
from geometry_msgs.msg import TransformStamped, PointStamped

import math
import numpy as np
import yaml
import cv2

class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('simple_slam')

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        #since map topic as it expects the map to be persistent for later subscirbers like rviz
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.grid_pub = self.create_publisher(OccupancyGrid, '/map', qos_profile)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_pose = None
        self.prev_pose = None

        self.map_resolution = 0.05  # meters per cell
        self.map_size = 1000  # Number of cells (assumes square grid)
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.origin = (self.map_size // 2, self.map_size // 2)  # Center of the map

    def scan_callback(self, msg):

        target_frame = 'odom'

        try:
            # Get the transform from the LaserScan frame to the target frame
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                self.get_clock().now(),  # Get latest available transform
                timeout=rclpy.time.Duration(seconds=1.0)
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return

        # Convert LaserScan to Cartesian coordinates
        points = self.laserscan_to_cartesian(msg, transform)

        # Update the map with transformed points
        for wx, wy, wz in points:
            mx = int((wx / self.map_resolution) + self.origin[0])
            my = int((wy / self.map_resolution) + self.origin[1])
            if 0 <= mx < self.map_size and 0 <= my < self.map_size:
                self.map[mx, my] = 100  # Mark as occupied

        self.get_logger().info(f'Scan received', once=True)
        self.publish_map()

    def publish_map(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'

        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_size
        grid_msg.info.height = self.map_size
        grid_msg.info.origin.position.x = -self.map_size // 2 * self.map_resolution
        grid_msg.info.origin.position.y = -self.map_size // 2 * self.map_resolution
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid = self.map.flatten()
        grid_msg.data = grid.tolist()
        self.grid_pub.publish(grid_msg)

    def save_map(self):
        # Save map as PNG and YAML files
        png_file = 'map.png'
        yaml_file = 'map.yaml'

        cv2.imwrite(png_file, self.map)
        with open(yaml_file, 'w') as f:
            yaml.dump({
                'image': png_file,
                'resolution': self.map_resolution,
                'origin': [-self.map_size // 2 * self.map_resolution, -self.map_size // 2 * self.map_resolution, 0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.2
            }, f)

    def laserscan_to_cartesian(self, scan_msg, transform):
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Remove invalid points (NaN or Inf)
        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        # Polar to Cartesian
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        z = np.zeros_like(x)  # 2D LaserScan, so z is 0

        # Create homogeneous coordinates for the points
        points = np.vstack((x, y, z, np.ones_like(x)))

        # Transform points to the target frame using the TF2 transform
        transform_matrix = self.tf_to_matrix(transform)
        transformed_points = transform_matrix @ points

        return transformed_points[:3, :].T  # Return x, y, z as rows

    def tf_to_matrix(self, transform):
        # Extract translation and rotation from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Create translation matrix
        transform_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
        transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        return transform_matrix
    
    ## taken from https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
    def quaternion_from_euler(r, p, y):
        r /= 2.0
        p /= 2.0
        y /= 2.0
        ci = math.cos(r)
        si = math.sin(r)
        cj = math.cos(p)
        sj = math.sin(p)
        ck = math.cos(y)
        sk = math.sin(y)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q


def main(args=None):
    rclpy.init(args=args)
    slam_node = SimpleSLAM()
    rclpy.spin(slam_node)

    slam_node.get_logger().info(f'Trying to save map', once=True)
    slam_node.save_map()
    slam_node.get_logger().info(f'Map saved!', once=True)

    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
