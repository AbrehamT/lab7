#!/usr/bin/env python3
import os
import glob
import csv
import math
from time import gmtime, strftime
from os.path import expanduser

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class RacelineGenerator(Node):
    def __init__(self):
        super().__init__('generate_raceline')

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
        self.get_logger().info(f'Loading raw waypoints from: {log_file}')

        # Read raw x, y, yaw, speed from CSV
        raw = []
        with open(log_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 4:
                    continue
                x, y = float(row[0]), float(row[1])
                yaw   = float(row[2])
                speed = float(row[3])
                raw.append((x, y, yaw, speed))

        # Downsample and apply dynamic speed filter
        window = 20
        self.waypoints = []
        for i in range(0, len(raw), window):
            w = raw[i:i+window]
            if not w:
                continue

            avg_x = sum(p[0] for p in w) / len(w)
            avg_y = sum(p[1] for p in w) / len(w)

            sin_sum = sum(math.sin(p[2]) for p in w)
            cos_sum = sum(math.cos(p[2]) for p in w)
            avg_yaw = math.atan2(sin_sum, cos_sum)

            # Apply curvature-aware weighting for speed
            curvature_weights = []
            for j in range(len(w)):
                if 0 < j < len(w)-1:
                    dx1 = w[j][0] - w[j-1][0]
                    dy1 = w[j][1] - w[j-1][1]
                    dx2 = w[j+1][0] - w[j][0]
                    dy2 = w[j+1][1] - w[j][1]
                    angle1 = math.atan2(dy1, dx1)
                    angle2 = math.atan2(dy2, dx2)
                    dtheta = abs(angle2 - angle1)
                    curvature = min(dtheta, 2 * math.pi - dtheta)
                    weight = max(0.5, 1.0 - curvature)  # weight ↓ with more curvature
                else:
                    weight = 1.0
                curvature_weights.append(weight)

            total_weight = sum(curvature_weights)
            avg_speed = sum(p[3] * w for p, w in zip(w, curvature_weights)) / total_weight
            avg_speed *= 1.1  # Boost straight segments slightly

            self.waypoints.append((avg_x, avg_y, avg_yaw, avg_speed))

        if not self.waypoints:
            self.get_logger().error('No valid points after downsampling')
            rclpy.shutdown()
            return

        # Publisher (latched)
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PoseArray, 'waypoint_poses', latch_qos)

        # Build & publish PoseArray
        pa = PoseArray()
        pa.header.frame_id = 'map'
        for x, y, yaw, speed in self.waypoints:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = speed  # embed speed in z
            q = quaternion_from_euler(0, 0, yaw)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            pa.poses.append(p)

        pa.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(pa)
        self.get_logger().info(
            f"Published {len(pa.poses)} waypoints w/ speed to /waypoint_poses"
        )

def main():
    rclpy.init()
    node = RacelineGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()