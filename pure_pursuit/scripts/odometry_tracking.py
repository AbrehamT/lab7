#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math
from transforms3d.euler import euler2quat
from std_msgs.msg import Float64

class OdometryTrackingNode(Node):
    def __init__(self):
        super().__init__('odometry_tracking')

        # init pose state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = None
        self.steering_angle = 0.0

        # constants (might need to be adjusted)
        self.wheelbase = 0.324  # wheelbase

        # subscribe to odometer and steering commands being sent
        self.create_subscription(Odometry, '/vesc/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/vesc/sensors/servo_position_command', self.servo_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/global_pose', 10)

        self.get_logger().info('odometry_tracking node initialized.')

    # calc car's global position and orientation based on velocity and steering angle
    def odom_callback(self, msg):
        # get current time 
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        # calculate time elapsed
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # get velocity and steering angle
        v = msg.twist.twist.linear.x
        delta = self.steering_angle

        # calc change in position and orientation
        dx = v * math.cos(self.theta) * dt
        dy = v * math.sin(self.theta) * dt
        dtheta = v / self.wheelbase * math.tan(delta) * dt

        # update pose state
        self.x += dx
        self.y += dy
        self.theta += dtheta

        # normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.get_logger().info(f"x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.theta):.2f}°")

        # publish PoseStamped
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        q = euler2quat(0, 0, self.theta)
        pose.pose.orientation.x = q[1]
        pose.pose.orientation.y = q[2]
        pose.pose.orientation.z = q[3]
        pose.pose.orientation.w = q[0]
        self.pose_pub.publish(pose)

    # convert servo signal to a usable steering angle (radians)
    def servo_callback(self, msg):
        servo_value = msg.data # ranges from 0.0 (hard right) to 1.0 (hard left)

        # constants (might need to be changed)
        max_steering_angle = 0.4189  # ~24 degrees in radians
        center_value = 0.5 # neutral (straight ahead)
        max_deviation = 0.34  # max deviation from center we can get

        # normalize
        normalized = (servo_value - center_value) / max_deviation
        normalized = max(-1.0, min(1.0, normalized))  # clamp values

        self.steering_angle = normalized * max_steering_angle

        self.get_logger().info(f"Steering angle: {math.degrees(self.steering_angle):.2f}°")


def main(args=None):
    rclpy.init(args=args)
    node = OdometryTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
