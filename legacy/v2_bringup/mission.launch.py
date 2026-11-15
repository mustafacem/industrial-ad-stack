"""Closed-loop autonomy: sim -> (perception) -> (localization) -> planner -> controller, driven by the mission
manager through a list of named site goals. Used by the planning, control and full-mission demos.

  ros2 launch iad_bringup mission.launch.py scenario:=quarry start_pose:=dump_approach goals:=dump
  ros2 launch iad_bringup mission.launch.py scenario:=quarry goals:=load,dump perception:=true localization:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = {
    'scenario': 'quarry', 'start_pose': 'start', 'goals': 'dump', 'dwell': '6.0', 'loaded': 'false',
    'algorithm': 'mpc', 'delay_compensation': 'true', 'grade_feedforward': 'true', 'actuator_delay': '-1.0',
    'perception': 'false', 'localization': 'false', 'speed': '6.0', 'rviz': 'true', 'rviz_config': 'planning.rviz',
    'hud': 'planning', 'start_paused': 'false', 'duration': '-1.0', 'log': '/ws/output/logs/mission.jsonl',
    'time_scale': '1.0', 'ego_dust': 'true', 'planning_obstacles': 'false',
}


def setup(context):
    a = {k: context.launch_configurations[k] for k in ARGS}
    on = lambda k: a[k].lower() in ('1', 'true', 'yes')  # noqa: E731
    st = {'use_sim_time': True}
    port = a['scenario'] == 'port'
    vehicle = 'container_agv' if port else 'haul_truck'
    odom = '/localization/odometry' if on('localization') else '/sim/ground_truth/odom'
    nodes = [
        Node(package='iad_sim', executable='sim_node', name='iad_sim', output='screen', on_exit=Shutdown(),
             additional_env={'OMP_NUM_THREADS': '4'},
             parameters=[{'scenario': a['scenario'], 'start_pose': a['start_pose'], 'command_source': 'external',
                          'loaded': on('loaded'), 'actuator_delay': float(a['actuator_delay']),
                          'publish_base_link_tf': not on('localization'), 'ego_ghost': on('localization'),
                          'start_paused': on('start_paused'), 'duration': float(a['duration']),
                          'time_scale': float(a['time_scale']), 'ego_dust': on('ego_dust'),
                          'planning_obstacles': on('planning_obstacles')}]),
        Node(package='iad_planning', executable='planner_node', name='planner', output='screen',
             parameters=[st, {'vehicle': vehicle, 'odom_topic': odom, 'use_live_occupancy': on('perception'),
                              'v_forward': float(a['speed'])}]),
        Node(package='iad_control', executable='controller_node', name='controller', output='screen',
             parameters=[st, {'vehicle': vehicle, 'algorithm': a['algorithm'], 'odom_topic': odom,
                              'delay_compensation': on('delay_compensation'), 'grade_feedforward': on('grade_feedforward'),
                              'actuator_delay': float(a['actuator_delay'])}]),
        Node(package='iad_tools', executable='mission', name='mission', output='screen',
             parameters=[st, {'goals': a['goals'], 'dwell': float(a['dwell'])}]),
        Node(package='iad_tools', executable='hud', name='hud', parameters=[{'profile': a['hud']}]),
        Node(package='iad_tools', executable='metrics_logger', name='metrics_logger',
             parameters=[st, {'path': a['log'], 'topics': ['/planning/metrics', '/control/metrics', '/sim/debug',
                                                          '/mission/status', '/perception/metrics', '/localization/metrics']}]),
    ]
    if on('perception'):
        nodes.append(Node(package='iad_perception', executable='perception_node', name='perception', namespace='perception',
                          output='screen', parameters=[st, {'mode': 'ours', 'vehicle': vehicle}]))
    if on('localization'):
        nodes.append(Node(package='iad_localization', executable='localization_node', name='localization', output='screen',
                          parameters=[st, {'vehicle': vehicle, 'init_error': [0.0, 0.0, 0.0]}]))
    if on('rviz'):
        cfg = FindPackageShare('iad_bringup').perform(context) + '/rviz/' + a['rviz_config']
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz', output='log', arguments=['-d', cfg],
                          parameters=[st], additional_env={'LP_NUM_THREADS': '6'}))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] +
                             [OpaqueFunction(function=setup)])
