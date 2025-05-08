#!/usr/bin/env python3

import os
import glob
import csv
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # vehicle & control parameters
        self.declare_parameter('wheelbase', 0.3)              # [m]
        self.declare_parameter('max_steering_angle', 0.4189) # [rad]
        self.declare_parameter('min_lookahead', 0.5)         # L_min [m]
        self.declare_parameter('lookahead_gain', 0.45)        # k_v [m per (m/s)]

        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_delta = self.get_parameter('max_steering_angle').value
        self.Ld_min    = self.get_parameter('min_lookahead').value
        self.k_v       = self.get_parameter('lookahead_gain').value

        # load the latest raceline CSV of (x, y, speed)
        home = os.path.expanduser('~')
        files = sorted(glob.glob(f"{home}/rcws/logs/raceline-*.csv"))
        if not files:
            self.get_logger().error("No raceline CSV found in ~/rcws/logs"); rclpy.shutdown(); return
        path = files[-1]

        self.speeds = []
        with open(path, 'r') as f:
            for row in csv.reader(f):
                # row = [x, y, speed]
                self.speeds.append(float(row[2]))
        self.get_logger().info(f"Loaded {len(self.speeds)} speeds from {path}")

        # state
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0
        self.path_pts    = []  # list of (x,y)

        # publishers & subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(PoseStamped, '/pf/viz/inferred_pose',
                                 self.pose_cb, 10)
        self.create_subscription(Path, '/interpolated_path',
                                 self.path_cb, 10)

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.current_x = p.x
        self.current_y = p.y
        siny = 2*(o.w*o.z + o.x*o.y)
        cosy = 1 - 2*(o.y*o.y + o.z*o.z)
        self.current_yaw = math.atan2(siny, cosy)

    def path_cb(self, msg: Path):
        # cache the geometry
        self.path_pts = [(p.pose.position.x, p.pose.position.y)
                         for p in msg.poses]
        self.run_pursuit()

    def run_pursuit(self):
        if not self.path_pts:
            return

        # 1) pick the drive speed by nearest‐point lookup
        dists = [((px-self.current_x)**2 + (py-self.current_y)**2, idx)
                 for idx, (px,py) in enumerate(self.path_pts)]
        _, nearest_idx = min(dists, key=lambda t: t[0])
        speed_cmd = self.speeds[nearest_idx]

        # 2) dynamic look‐ahead
        Ld = self.Ld_min + self.k_v * speed_cmd

        # 3) log for debug
        self.get_logger().info(
            f"Speed_cmd={speed_cmd:.2f} m/s, dynamic Ld={Ld:.2f} m"
        )

        # 4) find the lookahead goal point
        goal = self.find_goal(Ld)
        if goal is None:
            goal = self.path_pts[-1]

        # 5) transform into vehicle frame
        dx, dy = goal[0] - self.current_x, goal[1] - self.current_y
        x_v =  math.cos(self.current_yaw)*dx + math.sin(self.current_yaw)*dy
        y_v = -math.sin(self.current_yaw)*dx + math.cos(self.current_yaw)*dy

        # 6) compute steering
        curvature = 2.0 * y_v / (Ld**2)
        delta = math.atan(curvature * self.wheelbase)
        delta = max(-self.max_delta, min(self.max_delta, delta))

        # 7) publish drive command
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = delta
        cmd.drive.speed          = speed_cmd + 0.45
        self.drive_pub.publish(cmd)

    def find_goal(self, Ld):
        """Walk forward along path_pts until accumulating distance ≥ Ld."""
        cx, cy = self.current_x, self.current_y
        acc = 0.0
        # find nearest index again
        dists = [((px-cx)**2 + (py-cy)**2, i)
                 for i,(px,py) in enumerate(self.path_pts)]
        _, ni = min(dists, key=lambda t: t[0])

        for i in range(ni, len(self.path_pts)-1):
            x1,y1 = self.path_pts[i]
            x2,y2 = self.path_pts[i+1]
            seg = math.hypot(x2-x1, y2-y1)
            if acc + seg >= Ld:
                rem = Ld - acc
                frac = rem/seg
                return (x1 + frac*(x2-x1), y1 + frac*(y2-y1))
            acc += seg
        return None

def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
