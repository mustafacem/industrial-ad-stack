"""Simulator + RViz only (world / sensor check)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    a = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='quarry'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='quarry_view.rviz'),
        DeclareLaunchArgument('start_paused', default_value='false'),
        DeclareLaunchArgument('route', default_value='haul'),
        Node(package='iad_sim', executable='sim_node', name='iad_sim', output='screen',
             parameters=[{'scenario': a('scenario'), 'start_paused': a('start_paused'),
                          'autopilot_route': a('route')}]),
        Node(package='rviz2', executable='rviz2', name='rviz2', condition=IfCondition(a('rviz')),
             arguments=['-d', PathJoinSubstitution([FindPackageShare('iad_bringup'), 'rviz', a('rviz_config')])],
             parameters=[{'use_sim_time': True}]),
    ])
