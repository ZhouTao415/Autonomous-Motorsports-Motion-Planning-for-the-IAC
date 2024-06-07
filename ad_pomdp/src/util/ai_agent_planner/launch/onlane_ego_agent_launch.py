from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arena_info_topic', default_value='/arena_info'),
        DeclareLaunchArgument('arena_info_static_topic', default_value='/arena_info_static'),
        DeclareLaunchArgument('arena_info_dynamic_topic', default_value='/arena_info_dynamic'),
        DeclareLaunchArgument('global_desired_vel', default_value='10.0'),
        DeclareLaunchArgument('global_autonomous_level', default_value='3'),
        DeclareLaunchArgument('playground', default_value='highway_v1.0'),
        
        Node(
            package='ai_agent_planner',
            executable='onlane_ai_agent',
            name='onlane_ai_agent_0',
            output='screen',
            parameters=[{
                'ego_id': 0,
                'agent_config_path': PathJoinSubstitution([
                    FindPackageShare('playgrounds'),
                    LaunchConfiguration('playground'),
                    'agent_config.json'
                ]),
                'desired_vel': LaunchConfiguration('global_desired_vel'),
                'autonomous_level': LaunchConfiguration('global_autonomous_level'),
            }],
            remappings=[
                ('~arena_info', LaunchConfiguration('arena_info_topic')),
                ('~arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('~arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                ('~ctrl', '/ctrl/agent_0'),
            ]
        )
    ])
