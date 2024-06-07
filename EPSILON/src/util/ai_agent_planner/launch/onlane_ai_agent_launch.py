import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Define arguments
    args = [
        DeclareLaunchArgument('arena_info_topic', default_value='/arena_info'),
        DeclareLaunchArgument('arena_info_static_topic', default_value='/arena_info_static'),
        DeclareLaunchArgument('arena_info_dynamic_topic', default_value='/arena_info_dynamic'),
        DeclareLaunchArgument('global_desired_vel', default_value='10.0'),
        DeclareLaunchArgument('global_autonomous_level', default_value='2'),
        DeclareLaunchArgument('global_aggressiveness_level', default_def='4'),
        DeclareLaunchArgument('playground', default_value='highway_lite'),
    ]

    # Nodes
    nodes = []
    for i in range(1, 11):
        node = Node(
            package='ai_agent_planner',
            executable='onlane_ai_agent',
            name=f'onlane_ai_agent_{i}',
            output='screen',
            parameters=[{
                'ego_id': i,
                'agent_config_path': os.path.join(get_package_share_directory('playgrounds'), LaunchConfiguration('playground'), 'agent_config.json'),
                'desired_vel': LaunchConfiguration('global_desired_vel'),
                'autonomous_level': LaunchConfiguration('global_autonomous_level'),
                'aggressiveness_level': LaunchConfiguration('global_aggressiveness_level'),
            }],
            remappings=[
                ('~arena_info', LaunchConfiguration('arena_info_topic')),
                ('~arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('~arena_info_dynamic', LaunchConfiguration('arena_up_dynamic_topic')),
                ('~ctrl', f'/ctrl/agent_{i}')
            ]
        )
        nodes.append(node)

    return LaunchDescription(args + nodes)
