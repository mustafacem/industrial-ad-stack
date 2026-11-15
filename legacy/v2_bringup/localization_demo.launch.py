"""Localization demo (container terminal): GNSS multipath between stacks + outage under the cranes.
EKF + NDT (this stack) against gated GNSS, raw GNSS and dead reckoning, all on the same data."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    a = LaunchConfiguration
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_paused', default_value='false'),
        DeclareLaunchArgument('duration', default_value='-1.0'),
        DeclareLaunchArgument('log', default_value='/ws/output/logs/localization.jsonl'),
        Node(package='iad_sim', executable='sim_node', name='iad_sim', output='screen', on_exit=Shutdown(),
             additional_env={'OMP_NUM_THREADS': '6'},
             parameters=[{'scenario': 'port', 'autopilot_route': 'loc_loop', 'autopilot_speed': 7.0,
                          'publish_base_link_tf': False, 'ego_ghost': True, 'start_paused': a('start_paused'),
                          'duration': a('duration')}]),
        Node(package='iad_localization', executable='localization_node', name='localization', output='screen',
             parameters=[sim_time, {'vehicle': 'container_agv'}]),
        Node(package='iad_tools', executable='hud', name='hud', parameters=[{'profile': 'localization'}]),
        Node(package='iad_tools', executable='metrics_logger', name='metrics_logger',
             parameters=[sim_time, {'path': a('log'),
                                    'topics': ['/localization/metrics', '/sim/debug', '/sensing/gnss/status']}]),
        Node(package='rviz2', executable='rviz2', name='rviz', condition=IfCondition(a('rviz')), output='log',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('iad_bringup'), 'rviz', 'localization.rviz'])],
             parameters=[sim_time], additional_env={'LP_NUM_THREADS': '6'}),
    ])
