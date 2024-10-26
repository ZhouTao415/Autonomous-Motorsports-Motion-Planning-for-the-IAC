from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arena_info_static_topic', default_value='/arena_info_static'
        ),
        DeclareLaunchArgument(
            'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
        ),
        DeclareLaunchArgument(
            'ctrl_topic', default_value='/ctrl/agent_0'
        ),
        DeclareLaunchArgument(
            'playground', default_value='highway_lite'
        ),

        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['ctrl_topic: ', LaunchConfiguration('ctrl_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching node..."),

        Node(
            package='planning_integrated',
            executable='test_ssc_with_mpdm',
            name='test_ssc_with_mpdm_0',
            output='screen',
            parameters=[{
                'ego_id': 0,
                'desired_vel': 60.0,
                'use_sim_state': True,
                'agent_config_path': PathJoinSubstitution([
                    get_package_share_directory('playgrounds'),
                    LaunchConfiguration('playground'),
                    'agent_config.json'
                ]),
                'ssc_config_path': PathJoinSubstitution([
                    get_package_share_directory('ssc_planner'),
                    'config',
                    'ssc_config.pb.txt'
                ])
            }],
            remappings=[
                ('arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                ('ctrl', LaunchConfiguration('ctrl_topic'))
            ]
        )
    ])
