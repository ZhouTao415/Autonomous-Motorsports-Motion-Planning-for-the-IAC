from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arena_info_static_topic', default_value='/arena_info_static'),
        DeclareLaunchArgument('arena_info_dynamic_topic', default_value='/arena_info_dynamic'),
        DeclareLaunchArgument('ctrl_topic', default_value='/ctrl/agent_0'),
        DeclareLaunchArgument('playground', default_value='highway_v1.0'),

        Node(
            package='planning_integrated',
            executable='test_ssc_with_eudm',
            name='test_ssc_with_eudm_0',
            output='screen',
            parameters=[
                {'ego_id': 0},
                {'agent_config_path': Command(['find', 'playgrounds', '/', LaunchConfiguration('playground'), '/agent_config.json'])},
                {'bp_config_path': Command(['find', 'eudm_planner', '/config/eudm_config.pb.txt'])},
                {'ssc_config_path': Command(['find', 'ssc_planner', '/config/ssc_config.pb.txt'])},
                {'desired_vel': 20.0},
                {'use_sim_state': True}
            ],
            remappings=[
                ('~arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('~arena_es_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                ('~ctrl', LaunchConfiguration('ctrl_topic'))
            ]
        )
    ])
