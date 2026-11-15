"""Perception demo (quarry): the same LiDAR stream through the baseline and through this stack, side by side."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def rviz(cfg, name):
    return Node(package='rviz2', executable='rviz2', name=name, condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', PathJoinSubstitution([FindPackageShare('iad_bringup'), 'rviz', cfg])],
                parameters=[{'use_sim_time': True}], output='log', additional_env={'LP_NUM_THREADS': '4'})


def generate_launch_description():
    a = LaunchConfiguration
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_paused', default_value='false'),
        DeclareLaunchArgument('duration', default_value='-1.0'),
        DeclareLaunchArgument('log', default_value='/ws/output/logs/perception.jsonl'),
        Node(package='iad_sim', executable='sim_node', name='iad_sim', output='screen',
             parameters=[{'scenario': 'quarry', 'autopilot_route': 'haul', 'start_paused': a('start_paused'),
                          'duration': a('duration'), 'autopilot_speed': 6.5}], on_exit=Shutdown(),
             additional_env={'OMP_NUM_THREADS': '6'}),
        Node(package='iad_perception', executable='perception_node', name='perception', namespace='perception',
             parameters=[sim_time, {'mode': 'ours'}], output='screen'),
        Node(package='iad_perception', executable='perception_node', name='perception', namespace='perception_naive',
             parameters=[sim_time, {'mode': 'naive'}], output='screen'),
        Node(package='iad_tools', executable='hud', name='hud', parameters=[{'profile': 'perception'}]),
        Node(package='iad_tools', executable='metrics_logger', name='metrics_logger',
             parameters=[sim_time, {'path': a('log'),
                                    'topics': ['/perception/metrics', '/perception_naive/metrics', '/sim/debug']}]),
        rviz('perception_left.rviz', 'rviz_left'),
        rviz('perception_right.rviz', 'rviz_right'),
    ])
