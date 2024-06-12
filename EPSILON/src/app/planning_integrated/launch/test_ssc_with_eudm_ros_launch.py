from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
from launch.actions import LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
   # args that can be set from the command line or a default will be used
   arena_info_static_topic =  DeclareLaunchArgument(
       'arena_info_static_topic', default_value='/arena_info_static',
   )
   arena_info_dynamic_topic = DeclareLaunchArgument(
       'arena_info_dynamic_topic', default_value='/arena_info_dynamic'
   )
   ctrl_topic = DeclareLaunchArgument(
       'ctrl_topic' ,default_value='/ctrl/agent_0'
   )
   playground = DeclareLaunchArgument(
       'playground', default_value='highway_v1.0'
    #    'playground', default_value='ring_tiny_v1.0'
    #    'playground', default_value='ring_small_v1.0'
       
   )
    
   node = Node(
       package='planning_integrated',
       executable='test_ssc_with_eudm',
       name='test_ssc_with_eudm_0',
       output = 'screen',
       
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
       node
   ])



