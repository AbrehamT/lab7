#!/usr/bin/env python3

import os
import csv
from time import gmtime, strftime

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.interpolate import splprep, splev

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Header

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PathInterpolator(Node):
    def __init__(self):
        super().__init__('path_interpolator')

        # QoS to match the latched /waypoint_poses topic
        latch = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- CORRECTED: subscribe to PoseArray, not to PoseStamped.__class__ ---
        self.create_subscription(
            PoseArray,
            'waypoint_poses',
            self.waypoints_cb,
            latch
        )

        # publisher for the interpolated nav_msgs/Path
        self.pub = self.create_publisher(Path, '/interpolated_path', 10)
        self.timer = self.create_timer(0.5, self.publish_path)

        # storage for the last spline fit
        self.xy_interp = None   # Nx2 array of x,y
        self.s_interp  = None   # length‑N array of speed

        self.get_logger().info('PathInterpolator up, waiting for PoseArray…')

    def waypoints_cb(self, msg: PoseArray):
        # extract x, y, speed=z
        arr = np.array([[p.position.x, p.position.y, p.position.z]
                        for p in msg.poses])
        if arr.shape[0] < 2:
            self.get_logger().warn("Need ≥2 pts to interpolate")
            return

        # fit a 3‑D spline (x, y, speed)
        k = min(3, arr.shape[0] - 1)
        try:
            tck, _ = splprep([arr[:,0], arr[:,1], arr[:,2]], s=0, k=k)
        except Exception as e:
            self.get_logger().error(f"Spline prep failed: {e}")
            return

        # sample it finely
        N = 500
        u = np.linspace(0, 1, N)
        xi, yi, si = splev(u, tck)

        self.xy_interp = np.column_stack((xi, yi))
        self.s_interp  = si
        self.get_logger().info(f"Interpolated to {N} points.")

        # write out CSV once
        fn = strftime('raceline-%Y-%m-%d-%H-%M-%S.csv', gmtime())

        # ======================= LOGS_SLOW ONLY =======================

        out = os.path.join(home, 'sim_ws/src/pure_pursuit/rcws', 'logs_slow', 'wp-*.csv')
 
        # ======================= LOGS_FAST ONLY =======================

        # out = os.path.expanduser(f'~/final_files/rcws/logs_fast/{fn}')

        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            writer = csv.writer(f)
            for x_, y_, s_ in zip(xi, yi, si):
                writer.writerow([f"{x_:.6f}", f"{y_:.6f}", f"{s_:.6f}"])
        self.get_logger().info(f"Saved raceline CSV to {out}")

    def publish_path(self):
        if self.xy_interp is None:
            return

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for x_, y_ in self.xy_interp:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x_
            ps.pose.position.y = y_
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub.publish(path)

    def destroy_node(self):
        self.get_logger().info("Shutting down PathInterpolator")
        super().destroy_node()


def main():
    rclpy.init()
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
