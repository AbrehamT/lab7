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

        # — find most recent raw waypoint log —
        self.declare_parameter('log_file', '')
        log_file = self.get_parameter('log_file').value
        if not log_file:
            home = expanduser('~')
            # ======================= LOGS_SLOW ONLY =======================

            pattern = os.path.join(home, 'sim_ws/src/pure_pursuit/rcws', 'logs_slow', 'wp-*.csv')
        
            # ======================= LOGS_FAST ONLY =======================

            # pattern = os.path.join(home, 'final_files/rcws', 'logs_fast', 'wp-*.csv')
            files = sorted(glob.glob(pattern))
            if not files:
                self.get_logger().error(f'No log files found in {pattern}')
                rclpy.shutdown()
                return
            log_file = files[-1]
        self.get_logger().info(f'Loading raw waypoints from: {log_file}')

        # — read & window‑average (downsample) x,y,yaw,speed —
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

        window = 20
        self.waypoints = []
        for i in range(0, len(raw), window):
            w = raw[i:i+window]
            if not w:
                continue
            avg_x = sum(p[0] for p in w) / len(w)
            avg_y = sum(p[1] for p in w) / len(w)
            # circular mean for yaw
            sin_sum = sum(math.sin(p[2]) for p in w)
            cos_sum = sum(math.cos(p[2]) for p in w)
            avg_yaw = math.atan2(sin_sum, cos_sum)
            avg_speed = sum(p[3] for p in w) / len(w)
            self.waypoints.append((avg_x, avg_y, avg_yaw, avg_speed))

        if not self.waypoints:
            self.get_logger().error('No valid points after downsampling')
            rclpy.shutdown()
            return

        # — publisher (latched) —
        latch_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PoseArray, 'waypoint_poses', latch_qos)

        # — build & publish PoseArray —
        pa = PoseArray()
        pa.header.frame_id = 'map'
        for x, y, yaw, speed in self.waypoints:
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = speed      # ← embed speed here
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
        # clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
