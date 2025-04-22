#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # parameters
        self.declare_parameter('wheelbase', 0.3)             # meters
        self.declare_parameter('lookahead_distance', 1.25)   # meters
        self.declare_parameter('max_steering_angle', 0.4189)# radians (~24°)
        # self.declare_parameter('max_steering_angle', np.radians(50))# radians (~24°)

        self.declare_parameter('speed', 1.5)                 # m/s

        self.wheelbase = self.get_parameter('wheelbase').value
        self.Ld        = self.get_parameter('lookahead_distance').value
        self.max_delta = self.get_parameter('max_steering_angle').value
        self.speed     = self.get_parameter('speed').value

        # state
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0
        self.path_pts    = []

        # publishers & subscribers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(PoseStamped, '/pf/viz/inferred_pose', self.pose_callback, 10)
        self.create_subscription(Path,        '/interpolated_path', self.path_callback, 10)

    def pose_callback(self, msg: PoseStamped):
        pos = msg.pose.position
        o   = msg.pose.orientation
        self.current_x = pos.x
        self.current_y = pos.y
        # yaw from quaternion
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self.current_yaw = math.atan2(siny, cosy)

    def path_callback(self, msg: Path):
        # convert Path to list of (x,y)
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.run_pursuit()

    def run_pursuit(self):
        if not self.path_pts:
            return

        # find goal via walk-along approach
        goal = self.find_goal_point_walk()
        if goal is None:
            # fallback: last path point
            goal = self.path_pts[-1]

        # transform goal into vehicle frame
        dx = goal[0] - self.current_x
        dy = goal[1] - self.current_y
        x_v =  math.cos(self.current_yaw)*dx + math.sin(self.current_yaw)*dy
        y_v = -math.sin(self.current_yaw)*dx + math.cos(self.current_yaw)*dy

        # pure pursuit curvature & steering
        curvature = 2.0 * y_v / (self.Ld ** 2)
        delta = math.atan(curvature * self.wheelbase)
        # clamp
        delta = max(-self.max_delta, min(self.max_delta, delta))

        # publish drive command
        cmd = AckermannDriveStamped()
        cmd.drive.steering_angle = delta
        cmd.drive.speed          = self.speed
        self.drive_pub.publish(cmd)

    def find_goal_point_walk(self):
        """
        Walk forward along the pre-interpolated path from the nearest point
        until accumulating lookahead distance Ld. Returns (x, y) of that point.
        """
        cx, cy, Ld = self.current_x, self.current_y, self.Ld
        pts = self.path_pts

        # 1) Find nearest index
        dists = [( (px-cx)**2 + (py-cy)**2, idx ) for idx,(px,py) in enumerate(pts)]
        _, nearest_idx = min(dists, key=lambda x: x[0])

        # 2) Walk forward accumulating distance
        acc_dist = 0.0
        for i in range(nearest_idx, len(pts)-1):
            x1, y1 = pts[i]
            x2, y2 = pts[i+1]
            seg_dist = math.hypot(x2 - x1, y2 - y1)

            if acc_dist + seg_dist >= Ld:
                # need only part of this segment
                remain = Ld - acc_dist
                ratio = remain / seg_dist
                ix = x1 + ratio * (x2 - x1)
                iy = y1 + ratio * (y2 - y1)
                return (ix, iy)

            acc_dist += seg_dist

        # if we exit loop, the lookahead is beyond final point
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
