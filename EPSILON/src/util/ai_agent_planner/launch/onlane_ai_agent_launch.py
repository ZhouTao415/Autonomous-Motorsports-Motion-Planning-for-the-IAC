from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    arena_info_topic = DeclareLaunchArgument('arena_info_topic', default_value='/arena_info')
    arena_info_static_topic = DeclareLaunchArgument('arena_info_static_topic', default_value='/arena_info_static')
    arena_info_dynamic_topic = DeclareLaunchArgument('arena_info_dynamic_topic', default_value='/arena_info_dynamic')
    global_desired_vel = DeclareLaunchArgument('global_desired_vel', default_value='10.0')
    global_autonomous_level = DeclareLaunchArgument('global_autonomous_level', default_value='2')
    global_aggressiveness_level = DeclareLaunchArgument('global_aggressiveness_level', default_value='4')
    playground = DeclareLaunchArgument('playground', default_value='highway_lite')

    # Create Node configurations
    nodes = []
    for i in range(1, 11):
        node = Node(
            package='ai_agent_planner',
            executable='onlane_ai_agent',
            name=f'onlane_ai_agent_{i}',
            output='screen',
            parameters=[
                {'ego_id': i},
                {'agent_config_path': [get_package_share_directory('playgrounds'), '/', LaunchConfiguration('playground'), '/agent_config.json']},
                {'desired_vel': LaunchConfiguration('global_desired_vel')},
                {'autonomous_level': LaunchConfiguration('global_autonomous_level')},
                {'aggressiveness_level': LaunchConfiguration('global_aggressiveness_level')}
            ],
            remappings=[
                ('~arena_info', LaunchConfiguration('arena_info_topic')),
                ('~arena_info_static', LaunchConfiguration('arena_info_static_topic')),
                ('~arena_info_dynamic', LaunchConfiguration('arena_info_dynamic_topic')),
                ('~ctrl', f'/ctrl/agent_{i}')
            ]
        )
        nodes.append(node)

    # Return the launch description
    return LaunchDescription([
        arena_info_topic,
        arena_info_static_topic,
        arena_info_dynamic_topic,
        global_desired_vel,
        global_autonomous_level,
        global_aggressiveness_level,
        playground,
        *nodes
    ])
