import sys
import rclpy
from rclpy.node import Node

from interactive_markers import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, PoseStamped

marker_poses = {}

def print_quaternion(name, orientation):
    print(f"{name} orientation (quaternion): x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")

def publish_pose_array():
    pose_array_msg = PoseArray()
    pose_array_msg.header.frame_id = 'map'
    pose_array_msg.header.stamp = node.get_clock().now().to_msg()
    pose_array_msg.poses = list(marker_poses.values())
    pose_array_pub.publish(pose_array_msg)

def processFeedback(feedback):
    p = feedback.pose.position
    o = feedback.pose.orientation
    marker_poses[feedback.marker_name] = feedback.pose
    print(f'{feedback.marker_name} moved to x={p.x}, y={p.y}, z={p.z}')
    print_quaternion(feedback.marker_name, o)
    publish_pose_array()

def create_movable_marker(x, y, server):
    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = 'map'
    int_marker.name = f'my_marker({x:.2f},{y:.2f})'
    int_marker.description = 'Simple 2-DOF Control'
    
    # set initial position
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.position.z = 0.0

    #Store initial pose in global dictionary
    marker_poses[int_marker.name] = int_marker.pose
    print(f"{int_marker.name} created at x={x}, y={y}")
    print_quaternion(int_marker.name, int_marker.pose.orientation)

    # create a grey box marker
    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.25
    box_marker.scale.y = 0.25
    box_marker.scale.z = 0.25
    box_marker.color.r = 1.0
    box_marker.color.g = 0.0
    box_marker.color.b = 0.0
    box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    box_control.always_visible = True
    box_control.markers.append(box_marker)

    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    # create a control which will move the box
    rotate_control = InteractiveMarkerControl()
    rotate_control.orientation.w = 1.0
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.y = 1.0
    rotate_control.orientation.z = 0.0
    rotate_control.name = 'move_xy'
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

    # add the control to the interactive marker
    int_marker.controls.append(rotate_control)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, feedback_callback=processFeedback)

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('raceline_markers')

    pose_array_pub = node.create_publisher(PoseArray, '/waypoint_array', 10)

    # create an interactive marker server on the namespace simple_marker
    server = InteractiveMarkerServer(node, 'raceline_markers')

    #Number 1
    create_movable_marker(-5.1, -0.6, server)
    #Numner 25
    create_movable_marker(-2.6, -0.64, server)
    #Number 51
    create_movable_marker(0.02, -0.64, server)
    #Numbet 75
    create_movable_marker(2.4, -0.64, server)
    #Number 101
    create_movable_marker(5.2, -0.6, server)
    #Number 125
    create_movable_marker(7.6, -0.4, server)
    #Number 151
    create_movable_marker(9.6, 1.18, server)
    #Number 175
    create_movable_marker(10.1, 3.58, server)
    #Number 201
    create_movable_marker(10.0, 6.2, server)
    #Number 225
    create_movable_marker(8.8, 8.3, server)
    #Number 251
    create_movable_marker(6.26, 9.07, server)
    #Number 275
    create_movable_marker(3.7, 9.1, server)
    #Number 301
    create_movable_marker(1.0, 9.2, server)
    #Number 325
    create_movable_marker(-1.3, 9.1, server)
    #Number 351
    create_movable_marker(-3.98, 9.16, server)
    #Number 375
    create_movable_marker(-6.4, 9.1, server)
    #Number 401
    create_movable_marker(-9.1, 9.1, server)
    #Number 425
    create_movable_marker(-11.6, 8.8, server)
    #Number 451
    create_movable_marker(-13.6, 7.24, server)
    #Number 475
    create_movable_marker(-14.1, 4.8, server)
    #Number 501
    create_movable_marker(-13.8, 2.2, server)
    #Number 525
    create_movable_marker(-12.7, .04, server)
    #Number 551
    create_movable_marker(-10.13, -0.61, server)


    # 'commit' changes and send to all clients
    server.applyChanges()

    #publish initial pose array
    publish_pose_array()

    rclpy.spin(node)
    server.shutdown()