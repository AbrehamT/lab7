import sys

from interactive_markers import InteractiveMarkerServer
import rclpy
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

marker_poses = {}
def print_quaternion(name, orientation):

    print(f"{name} orientation (quaternion): x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

def publish_pose_array():
    pose_array_msg = PoseArray()
    pose_array_msg.header.frame_id = 'map'
    pose_array_msg.header.stamp = node.get_clock().now().to_msg()
    # pose_array_msg.poses = [pose for pose in marker_poses.values()]
    pose_array_msg.poses = [marker_poses[name] for name in sorted(marker_poses.keys())]

    pose_array_pub.publish(pose_array_msg)

def processFeedback(feedback):
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = 'map'
    pose_msg.header.stamp = node.get_clock().now().to_msg()
    pose_msg.pose = feedback.pose
    pose_pub.publish(pose_msg)
    p = feedback.pose.position
    o = feedback.pose.orientation
    marker_poses[feedback.marker_name] = feedback.pose
    print(f'{feedback.marker_name} moved to x={p.x}, y={p.y}, z={p.z}')
    print_quaternion(feedback.marker_name, o)
    publish_pose_array()


def create_movable_marker(x, y, server):
    # Create interactive marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'map'
    int_marker.name = f'my_marker({x},{y})'
    int_marker.description = 'Simple 2-DOF Control'
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.position.z = 0.0

    # Store initial pose
    marker_poses[int_marker.name] = int_marker.pose
    print(f"{int_marker.name} created at x={x}, y={y}")
    print_quaternion(int_marker.name, int_marker.pose.orientation)

    # Create a red box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.25
    box_marker.scale.y = 0.25
    box_marker.scale.z = 0.25
    box_marker.color.r = 1.0
    box_marker.color.g = 0.0
    box_marker.color.b = 0.0
    box_marker.color.a = 1.0

    # Attach the marker to a non-interactive control for visualization
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)
    int_marker.controls.append(box_control)

    # Add MOVE_PLANE control
    move_control = InteractiveMarkerControl()
    move_control.name = "move_xy"
    move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    move_control.orientation.w = 1.0
    move_control.orientation.x = 0.0
    move_control.orientation.y = 1.0
    move_control.orientation.z = 0.0
    move_control.always_visible = True
    int_marker.controls.append(move_control)

    # Insert the interactive marker into the server
    server.insert(int_marker, feedback_callback=processFeedback)

def main():
    global pose_pub, pose_array_pub, node

    rclpy.init(args=sys.argv)
    node = rclpy.create_node('simple_marker')

    pose_pub = node.create_publisher(PoseStamped, '/current_pose', 10)
    pose_array_pub = node.create_publisher(PoseArray, '/waypoint_array', 10)

    server = InteractiveMarkerServer(node, 'simple_marker')

    create_movable_marker(-5.1, -0.6, server)
    create_movable_marker(0.02, -0.64, server)
    create_movable_marker(5.2, -0.6, server)
    create_movable_marker(9.6, 1.18, server)
    create_movable_marker(10.0, 6.2, server)
    create_movable_marker(6.26, 9.07, server)
    create_movable_marker(1.0, 9.2, server)
    create_movable_marker(-3.98, 9.16, server)
    create_movable_marker(-9.1, 9.1, server)
    create_movable_marker(-13.6, 7.24, server)
    create_movable_marker(-13.8, 2.2, server)
    create_movable_marker(-10.13, -0.61, server)

    server.applyChanges()
    rclpy.spin(node)
    server.shutdown()
if __name__ == '__main__':
    main()
