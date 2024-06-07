from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arena_info_static_topic', default_value='/arena_info_static',
            description='Static arena info topic'),
        DeclareLaunchArgument(
            'arena_info_dynamic_topic', default_value='/arena_info_dynamic',
            description='Dynamic arena info topic'),
        DeclareLaunchArgument(
            'playground', default_value='highway_v1.0',
            description='Playground version'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([FindExecutable('phy_simulator'), 'launch', 'joy_ctrl.launch'])])
        ),

        Node(
            package='phy_simulator',
            executable='phy_simulator_planning_node',
            name='phy_simulator_planning_node',
            output='screen',
            parameters=[
                {'vehicle_info_path': PathJoinSubstitution([
                    FindExecutable('playgrounds'), LaunchConfiguration('playground'), 'vehicle_set.json'
                ])},
                {'map_path': PathJoinSubstitution([
                    FindExecutable('playgrounds'), LaunchConfiguration('playground'), 'obstacles_norm.json'
                ])},
                {'lane_net_path': PathJoinSubstitution([
                    FindExecutable('playgrounds'), LaunchConfiguration('playgrid'), 'lane_net_norm.json'
                ])},
            ],
            remappings=[
                ('~arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('~arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
            ],
        ),
    ])
