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

        self.waypoints = self.create_subscription(
            PoseArray,
            '/waypoint_array',
            self.waypoints_callback,
            10
        )