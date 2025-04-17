# pure_pursuit/launch/pure_pursuit_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='race_line',
            name='race_line_node',
            output='screen'
        ),
        Node(
            package='pure_pursuit',
            executable='interpolator',
            name='path_interpolator_node',
            output='screen'
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen'
        )
    ])
