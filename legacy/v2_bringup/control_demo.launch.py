"""Control demo (quarry haul road, loaded truck, 0.4 s actuator dead time): track the surveyed haul-road
reference with the delay-compensated MPC or the pure-pursuit baseline (algorithm:=mpc|pure_pursuit)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    a = LaunchConfiguration
    sim_time = {'use_sim_time': True}
    gt = '/sim/ground_truth/odom'
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='control.rviz'),
        DeclareLaunchArgument('start_paused', default_value='false'),
        DeclareLaunchArgument('duration', default_value='-1.0'),
        DeclareLaunchArgument('algorithm', default_value='mpc'),
        DeclareLaunchArgument('delay_compensation', default_value='true'),
        DeclareLaunchArgument('grade_feedforward', default_value='true'),
        DeclareLaunchArgument('actuator_delay', default_value='0.4'),
        DeclareLaunchArgument('speed', default_value='8.0'),
        DeclareLaunchArgument('log', default_value='/ws/output/logs/control.jsonl'),
        Node(package='iad_sim', executable='sim_node', name='iad_sim', output='screen', on_exit=Shutdown(),
             additional_env={'OMP_NUM_THREADS': '4'},
             parameters=[{'scenario': 'quarry', 'command_source': 'external', 'autopilot_route': 'haul', 'loaded': True,
                          'actuator_delay': ParameterValue(a('actuator_delay'), value_type=float),
                          'start_paused': a('start_paused'), 'duration': a('duration')}]),
        Node(package='iad_planning', executable='planner_node', name='planner', output='screen',
             parameters=[sim_time, {'route_mode': True, 'odom_topic': gt, 'use_live_occupancy': False,
                                    'v_forward': ParameterValue(a('speed'), value_type=float)}]),
        Node(package='iad_control', executable='controller_node', name='controller', output='screen',
             parameters=[sim_time, {'algorithm': a('algorithm'), 'odom_topic': gt,
                                    'delay_compensation': ParameterValue(a('delay_compensation'), value_type=bool),
                                    'grade_feedforward': ParameterValue(a('grade_feedforward'), value_type=bool),
                                    'actuator_delay': ParameterValue(a('actuator_delay'), value_type=float)}]),
        Node(package='iad_tools', executable='hud', name='hud', parameters=[{'profile': 'control'}]),
        Node(package='iad_tools', executable='metrics_logger', name='metrics_logger',
             parameters=[sim_time, {'path': a('log'), 'topics': ['/control/metrics', '/sim/debug', '/planning/metrics']}]),
        Node(package='rviz2', executable='rviz2', name='rviz', condition=IfCondition(a('rviz')), output='log',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('iad_bringup'), 'rviz', a('rviz_config')])],
             parameters=[sim_time], additional_env={'LP_NUM_THREADS': '6'}),
    ])
