#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header

class PathInterpolator(Node):
    def __init__(self):
        super().__init__('path_interpolator')

        self.publisher_ = self.create_publisher(Path, '/interpolated_path', 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)

        # Subscriber handle
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            '/waypoint_array',
            self.waypoints_callback,
            10
        )

        # Initialize path data
        self.interpolated_path = []

    def waypoints_callback(self, msg):
        points = np.array([[pose.position.x, pose.position.y] for pose in msg.poses])
        self.get_logger().info(f"Received {len(points)} waypoints")
        self.interpolated_path = self.interpolate_linear(points, resolution=0.1)

    def interpolate_linear(self, waypoints, resolution=0.1):
        if len(waypoints) < 2:
            return []

        points = []
        for i in range(len(waypoints) - 1):
            p0, p1 = waypoints[i], waypoints[i + 1]
            dist = np.linalg.norm(p1 - p0)
            steps = int(np.ceil(dist / resolution))
            for j in range(steps):
                t = j / steps
                point = p0 + t * (p1 - p0)
                points.append(point)
        points.append(waypoints[-1])
        return np.array(points)

    def timer_callback(self):
        if len(self.interpolated_path) == 0:
            return  # No path yet

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
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info(f'Published path with {len(path_msg.poses)} points.')

def main(args=None):
    rclpy.init(args=args)
    node = PathInterpolator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
