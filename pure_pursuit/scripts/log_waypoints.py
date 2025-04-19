#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv, os
from os.path import expanduser
from time import gmtime, strftime
from tf_transformations import euler_from_quaternion
from numpy.linalg import norm

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg    import Odometry

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')

        # --- open CSV file ---
        home = expanduser('~')
        timestamp = strftime('wp-%Y-%m-%d-%H-%M-%S.csv', gmtime())
        log_dir = os.path.join(home, 'rcws', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, timestamp)
        self.csv_file = open(self.csv_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.get_logger().info(f"Logging to {self.csv_path}")

        # --- state holders ---
        self.latest_pose = None   # geometry_msgs/PoseStamped
        self.latest_odom = None   # nav_msgs/Odometry

        # --- subscriptions ---
        self.create_subscription(
            PoseStamped,
            '/pf/viz/inferred_pose',
            self.pf_pose_callback,
            10
        )
        self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

    def pf_pose_callback(self, msg: PoseStamped):
        """ Store the most recent particle-filter pose. """
        self.latest_pose = msg

    def odom_callback(self, msg: Odometry):
        """
        On each odom message, if we already have a PF pose,
        log x,y,yaw from the PF and speed from the odom.
        """
        self.latest_odom = msg

        if self.latest_pose is None:
            return  # haven’t got a PF pose yet

        # 1) extract yaw from PF pose
        q = self.latest_pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # 2) compute speed from odometry twist
        v = msg.twist.twist.linear
        speed = norm([v.x, v.y, v.z])

        # 3) get position from PF pose
        p = self.latest_pose.pose.position
        # 4) write to CSV
        self.csv_writer.writerow([f"{p.x:.6f}", f"{p.y:.6f}", f"{yaw:.6f}", f"{speed:.6f}"])
        self.csv_file.flush()
        self.get_logger().info(f"Logged WP: x={p.x:.2f}, y={p.y:.2f}, yaw={yaw:.2f}, v={speed:.2f}")

    def destroy_node(self):
        # close file before shutting down
        self.csv_file.close()
        self.get_logger().info("Closed CSV file, goodbye.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
