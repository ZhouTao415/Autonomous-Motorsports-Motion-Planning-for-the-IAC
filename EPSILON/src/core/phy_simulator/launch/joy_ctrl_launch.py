from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            namespace='',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}],
        )
    ])
