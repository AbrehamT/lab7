#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
from scipy.interpolate import splprep, splev
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PathInterpolator(Node):
    def __init__(self):
        super().__init__('path_interpolator')

        # QoS to match the latched /waypoint_poses topic
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Subscribe to waypoints
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            '/waypoint_poses',
            self.waypoints_callback,
            latched_qos
        )

        # Publisher for interpolated path
        self.publisher_ = self.create_publisher(Path, '/interpolated_path', 10)

        # Timer to periodically publish path
        self.timer_ = self.create_timer(0.5, self.timer_callback)

        # Storage for interpolated path
        self.interpolated_path = []

        self.get_logger().info('PathInterpolator node started.')

    def waypoints_callback(self, msg):
        points = np.array([[pose.position.x, pose.position.y] for pose in msg.poses])
        self.get_logger().info(f"Received {len(points)} waypoints")

        # Use spline interpolation
        self.interpolated_path = self.interpolate_spline(points, num_points=1000)

    def interpolate_spline(self, waypoints, num_points=500):
        if len(waypoints) < 2:
            self.get_logger().warn("Not enough waypoints to interpolate.")
            return []

        x = waypoints[:, 0]
        y = waypoints[:, 1]

        try:
            tck, _ = splprep([x, y], s=0, k=min(3, len(waypoints) - 1))
            u_fine = np.linspace(0, 1, num_points)
            x_interp, y_interp = splev(u_fine, tck)
            return np.column_stack((x_interp, y_interp))
        except Exception as e:
            self.get_logger().error(f"Spline interpolation failed: {e}")
            return []

    def timer_callback(self):
        if len(self.interpolated_path) == 0:
            return

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for pt in self.interpolated_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Facing forward
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} points.')

def main(args=None):
    rclpy.init(args=args)
    node = PathInterpolator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
