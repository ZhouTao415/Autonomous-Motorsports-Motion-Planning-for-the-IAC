from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    arena_info_static_topic = DeclareLaunchArgument(
        'arena_info_static_topic', default_value='/arena_info_static'
    )
    arena_info_dynamic_topic = DeclareLaunchArgument(
        'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
    )
    playground = DeclareLaunchArgument(
        'playground', default_value='highway_v1.0'
    )

    # Include joy_ctrl launch file
    joy_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('phy_simulator'),
                'launch',
                'joy_ctrl_launch.py'
            ])
        ])
    )

    vehicle_info_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'vehicle_set.json'
    ])
    map_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'obstacles_norm.json'
    ])
    lane_net_path = PathJoinSubstitution([
        get_package_share_directory('playgrounds'),
        LaunchConfiguration('playground'),
        'lane_net_norm.json'
    ])

    # Node configuration
    phy_simulator_planning_node = Node(
        package='phy_simulator',
        executable='phy_simulator_planning_node',
        name='phy_simulator_planning_node',
        output='screen',
        parameters=[{
            'vehicle_info_path': vehicle_info_path,
            'map_path': map_path,
            'lane_net_path': lane_net_path
        }]
        # remappings=[
        #     ('/phy_simulator/arena_info_static', LaunchConfiguration('arena_info_static_topic')),
        #     ('/phy_simulator/arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic'))
        # ]
    )

    return LaunchDescription([
        arena_info_static_topic,
        arena_info_dynamic_topic,
        playground,
        joy_ctrl_launch,
        LogInfo(msg=['arena_info_static_topic: ', LaunchConfiguration('arena_info_static_topic')]),
        LogInfo(msg=['arena_info_dynamic_topic: ', LaunchConfiguration('arena_info_dynamic_topic')]),
        LogInfo(msg=['playground: ', LaunchConfiguration('playground')]),
        LogInfo(msg=['vehicle_info_path: ', vehicle_info_path]),
        LogInfo(msg=['map_path: ', map_path]),
        LogInfo(msg=['lane_net_path: ', lane_net_path]),
        LogInfo(msg="Launching node..."),
        phy_simulator_planning_node
    ])
    