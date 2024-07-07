from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    arena_info_topic = DeclareLaunchArgument(
        'arena_info_topic', default_value='/arena_info'
    )
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    global_desired_vel = DeclareLaunchArgument(
        'global_desired_vel', default_value='10.0'
    )
    global_autonomous_level = DeclareLaunchArgument(
        'global_autonomous_level', default_value='3'
    )
    global_aggressiveness_level = DeclareLaunchArgument(
        'global_aggressiveness_level', default_value='4'
    )
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_lite'
    )

    onlane_ai_agent_nodes = []
    for i in range(1, 11):
        onlane_ai_agent_nodes.append(Node(
            package='ai_agent_planner',
            executable='onlane_ai_agent',
            name=f'onlane_ai_agent_{i}',
            output='screen',
            parameters=[{
                'ego_id': i,
                'agent_config_path': PathJoinSubstitution([
                    get_package_share_directory('playgrounds'),
                    LaunchConfiguration('playground'),
                    'agent_config.json'
                ]),
                'desired_vel': LaunchConfiguration('global_desired_vel'),
                'autonomous_level': LaunchConfiguration('global_autonomous_level'),
                'aggressiveness_level': LaunchConfiguration('global_aggressiveness_level')
            }],
            remappings=[
                ('arena_info', LaunchConfiguration('arena_info_topic')),
                ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                ('ctrl', f'/ctrl/agent_{i}')
            ]
        ))

    return LaunchDescription([
        arena_info_topic,
        arena_info_static_topic,
        arena_info_dynamic_topic,
        global_desired_vel,
        global_autonomous_level,
        global_aggressiveness_level,
        playground,
        LogInfo(msg=['arena_info_topic: ', LaunchConfiguration('arena_info_topic')]),
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['global_desired_vel: ', LaunchConfiguration('global_desired_vel')]),
        LogInfo(msg=['global_autonomous_level: ', LaunchConfiguration('global_autonomous_level')]),
        LogInfo(msg=['global_aggressiveness_level: ', LaunchConfiguration('global_aggressiveness_level')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching nodes..."),
        *onlane_ai_agent_nodes
    ])
