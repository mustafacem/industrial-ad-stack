"""Calibration demo (container terminal): 4 corner LiDARs on the AGV with wrong extrinsics (TF claims up to
2.5 deg / 20 cm off). Ground plane at standstill, then mapping-based registration against the roof LiDAR."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    a = LaunchConfiguration
    st = {'use_sim_time': True}
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_paused', default_value='false'),
        DeclareLaunchArgument('duration', default_value='-1.0'),
        DeclareLaunchArgument('log', default_value='/ws/output/logs/calibration.jsonl'),
        Node(package='iad_sim', executable='sim_node', name='iad_sim', output='screen', on_exit=Shutdown(),
             additional_env={'OMP_NUM_THREADS': '6'},
             parameters=[{'scenario': 'port', 'corner_lidars': True, 'extrinsic_error_deg': 2.5, 'extrinsic_error_m': 0.2,
                          'start_pose': 'calib_start', 'autopilot_route': 'calib_route', 'autopilot_speed': 3.0,
                          'autopilot_delay': 4.0, 'start_paused': a('start_paused'), 'duration': a('duration')}]),
        Node(package='iad_calibration', executable='calibration_node', name='calibration', output='screen',
             parameters=[st, {'output': '/ws/output/calibration/extrinsics.yaml', 'keyframe_spacing': 3.5,
                              'max_keyframes': 55, 'max_iterations': 40}]),
        Node(package='iad_tools', executable='hud', name='hud', parameters=[{'profile': 'calibration'}]),
        Node(package='iad_tools', executable='metrics_logger', name='metrics_logger',
             parameters=[st, {'path': a('log'), 'topics': ['/calibration/metrics', '/sim/debug']}]),
        Node(package='rviz2', executable='rviz2', name='rviz', condition=IfCondition(a('rviz')), output='log',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('iad_bringup'), 'rviz', 'calibration.rviz'])],
             parameters=[st], additional_env={'LP_NUM_THREADS': '6'}),
    ])
