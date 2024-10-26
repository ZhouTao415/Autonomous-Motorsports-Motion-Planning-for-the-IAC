from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Declare launch arguments
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    ctrl_topic = DeclareLaunchArgument(
        'ctrl_topic', default_value='/ctrl/agent_0'
    )
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # Define the nodes to be launched
    test_ssc_with_eudm_node = Node(
        package='planning_integrated',
        executable='test_ssc_with_eudm',
        name='test_ssc_with_eudm_0',
        output='screen',
        parameters=[{
            'ego_id': 0,
            'desired_vel': 20.0,
            'use_sim_state': True,
            'agent_config_path': PathJoinSubstitution([
                get_package_share_directory('playgrounds'),
                LaunchConfiguration('playground'),
                'agent_config.json'
            ]),
            'bp_config_path': PathJoinSubstitution([
                get_package_share_directory('eudm_planner'),
                'config',
                'eudm_config.pb.txt'
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

    return LaunchDescription([
        arena_info_static_topic,
        arena_info_dynamic_topic,
        ctrl_topic,
        playground,
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['ctrl_topic: ', LaunchConfiguration('ctrl_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg="Launching node..."),
        test_ssc_with_eudm_node
    ])
