#!/usr/bin/env python3
import os
import glob
import csv
from os.path import expanduser

import rclpy
from rclpy.node import Node

# interactive markers for ROS 2
from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from tf_transformations import quaternion_from_euler


class WaypointMarkerLoader(Node):
    def __init__(self):
        super().__init__('waypoint_marker_loader')

        # 1. Get log file path (CLI param or latest in ~/rcws/logs)
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

        # 2. Read CSV: each line = x, y, yaw, speed
        waypoints = []
        try:
            with open(log_file, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue
                    x, y = float(row[0]), float(row[1])
                    yaw = float(row[2]) if len(row) > 2 else 0.0
                    waypoints.append((x, y, yaw))
        except Exception as e:
            self.get_logger().error(f'Failed to read {log_file}: {e}')
            rclpy.shutdown()
            return

        if not waypoints:
            self.get_logger().error('No valid waypoints found in the file.')
            rclpy.shutdown()
            return

        # 3. Create the InteractiveMarkerServer
        self.server = InteractiveMarkerServer(self, 'waypoint_markers')

        # 4. For each waypoint, insert an interactive marker
        for idx, (x, y, yaw) in enumerate(waypoints):
            self._create_waypoint_marker(idx, x, y, yaw)

        # 5. Apply changes so they appear in RViz
        self.server.applyChanges()
        self.get_logger().info(f'Published {len(waypoints)} waypoint markers.')

    def _create_waypoint_marker(self, idx: int, x: float, y: float, yaw: float):
        # Create and configure the interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'map'
        int_marker.name = f'waypoint_{idx}'
        int_marker.description = f'WP {idx}'
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.position.z = 0.0

        # Set orientation from yaw
        q = quaternion_from_euler(0, 0, yaw)
        int_marker.pose.orientation.x = q[0]
        int_marker.pose.orientation.y = q[1]
        int_marker.pose.orientation.z = q[2]
        int_marker.pose.orientation.w = q[3]

        # Visual marker (green sphere)
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.3
        sphere.scale.y = 0.3
        sphere.scale.z = 0.3
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.0
        sphere.color.a = 1.0

        # A non-interactive control to display the sphere
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(sphere)
        int_marker.controls.append(control)

        # Insert into the server (no feedback callback, static markers)
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
