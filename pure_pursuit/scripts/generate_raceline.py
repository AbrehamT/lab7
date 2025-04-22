#!/usr/bin/env python3
import os
import glob
import csv
import math
from os.path import expanduser

import rclpy
from rclpy.node import Node

from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class WaypointMarkerLoader(Node):
    def __init__(self):
        super().__init__('waypoint_marker_loader')

        # Load waypoint CSV
        self.declare_parameter('log_file', '')
        log_file = self.get_parameter('log_file').value
        if not log_file:
            home = expanduser('~')
            pattern = os.path.join(home, 'rcws', 'logs', 'wp-*.csv')
            files = sorted(glob.glob(pattern))
            if not files:
                self.get_logger().error(f'No log files found in {pattern}')
                rclpy.shutdown()
                return
            log_file = files[-1]
        self.get_logger().info(f'Loading waypoints from: {log_file}')

        # Parse waypoints with downsampling and yaw smoothing
        try:
            with open(log_file, 'r') as f:
                reader = csv.reader(f)
                raw_points = []
                for row in reader:
                    if len(row) < 2:
                        continue
                    if float(row[2]) == 0.0:
                        continue
                    x, y = float(row[0]), float(row[1])
                    yaw = float(row[2]) if len(row) > 2 else 0.0
                    speed = float(row[3]) if len(row) > 3 else 0.0
                    raw_points.append((x, y, yaw, speed))

                window_size = 20
                self.waypoints = []
                for i in range(0, len(raw_points), window_size):
                    window = raw_points[i:i+window_size]
                    if not window:
                        continue
                    avg_x = sum(p[0] for p in window) / len(window)
                    avg_y = sum(p[1] for p in window) / len(window)
                    sin_sum = sum(math.sin(p[2]) for p in window)
                    cos_sum = sum(math.cos(p[2]) for p in window)
                    avg_yaw = math.atan2(sin_sum, cos_sum)
                    avg_speed = sum(p[3] for p in window) / len(window)

                    self.waypoints.append((avg_x, avg_y, avg_yaw, avg_speed))

        except Exception as e:
            self.get_logger().error(f'Failed to read {log_file}: {e}')
            rclpy.shutdown()
            return

        if not self.waypoints:
            self.get_logger().error('No valid waypoints found in the file.')
            rclpy.shutdown()
            return

        # Create publisher with TRANSIENT_LOCAL so it behaves like latched
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_pub = self.create_publisher(PoseArray, 'waypoint_poses', latch_qos)

        # Interactive markers for visualization
        self.server = InteractiveMarkerServer(self, 'waypoint_markers')

        # Build PoseArray and markers
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = 'map'
        for idx, (x, y, yaw) in enumerate(self.waypoints):
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            self.pose_array.poses.append(pose)
            self._create_waypoint_marker(idx, x, y, yaw)

        self.server.applyChanges()
        self.get_logger().info(f'Published {len(self.waypoints)} waypoint markers.')

        # Publish once with timestamp
        self.pose_array.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.pose_array)
        self.get_logger().info(f'PoseArray published once to /waypoint_poses (latched).')

    def _create_waypoint_marker(self, idx: int, x: float, y: float, yaw: float):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.name = f'waypoint_{idx}'
        int_marker.description = f'WP {idx}'
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, yaw)
        int_marker.pose.orientation.x = q[0]
        int_marker.pose.orientation.y = q[1]
        int_marker.pose.orientation.z = q[2]
        int_marker.pose.orientation.w = q[3]

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.3
        sphere.scale.y = 0.3
        sphere.scale.z = 0.3
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(sphere)
        int_marker.controls.append(control)

        self.server.insert(int_marker)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerLoader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
