#!/usr/bin/env python3
"""Generate the RViz2 configs (each subsystem package's own view + the demo views in iad_bringup).

RViz YAML is verbose and every demo shares the same building blocks (world meshes, ego, HUD), so the
configs are generated rather than hand-edited:  python3 tools/make_rviz.py
"""
import os

import yaml

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
# RViz window chrome (menu + toolbar + status bar + dock handles) around the 3D panel; the record
# script crops it away so the video is exactly the 3D view.
WIN_W, WIN_H = 1970, 1189


def qos(topic, depth=5, transient=False, best_effort=False):
    return {'Value': topic, 'Depth': depth,
            'Durability Policy': 'Transient Local' if transient else 'Volatile',
            'History Policy': 'Keep Last',
            'Reliability Policy': 'Best Effort' if best_effort else 'Reliable'}


def rgb(c):
    return '%d; %d; %d' % tuple(c)


def marker_array(name, topic, depth=64, transient=True, enabled=True):
    return {'Class': 'rviz_default_plugins/MarkerArray', 'Name': name, 'Enabled': enabled, 'Namespaces': {},
            'Topic': qos(topic, depth, transient), 'Value': enabled}


def marker(name, topic, depth=5, transient=False, enabled=True):
    return {'Class': 'rviz_default_plugins/Marker', 'Name': name, 'Enabled': enabled, 'Namespaces': {},
            'Topic': qos(topic, depth, transient), 'Value': enabled}


def cloud(name, topic, color='intensity', flat=(255, 255, 255), size_m=0.08, size_px=2, style='Flat Squares',
          alpha=1.0, imin=0, imax=100, channel='intensity', best_effort=True, transient=False, decay=0,
          enabled=True, zmin=-5, zmax=15):
    d = {'Class': 'rviz_default_plugins/PointCloud2', 'Name': name, 'Enabled': enabled, 'Alpha': alpha,
         'Autocompute Intensity Bounds': False, 'Min Intensity': imin, 'Max Intensity': imax,
         'Channel Name': channel,
         'Color Transformer': {'intensity': 'Intensity', 'flat': 'FlatColor', 'axis': 'AxisColor'}[color],
         'Color': rgb(flat), 'Decay Time': decay, 'Position Transformer': 'XYZ', 'Selectable': False,
         'Size (Pixels)': size_px, 'Size (m)': size_m, 'Style': style,
         'Topic': qos(topic, 2 if not transient else 1, transient, best_effort and not transient),
         'Use Fixed Frame': True, 'Use rainbow': True, 'Invert Rainbow': False, 'Value': enabled,
         'Max Color': '255; 255; 255', 'Min Color': '0; 0; 0'}
    if color == 'axis':
        d['Axis'] = 'Z'
        d['Autocompute Value Bounds'] = {'Max Value': zmax, 'Min Value': zmin, 'Value': False}
    return d


def path(name, topic, color, width=0.3, alpha=1.0, transient=False, style='Billboards', offset_z=0.0):
    return {'Class': 'rviz_default_plugins/Path', 'Name': name, 'Enabled': True, 'Alpha': alpha,
            'Buffer Length': 1, 'Color': rgb(color), 'Head Diameter': 0.3, 'Head Length': 0.2, 'Length': 0.3,
            'Line Style': style, 'Line Width': width, 'Offset': {'X': 0, 'Y': 0, 'Z': offset_z},
            'Pose Color': '255; 85; 255', 'Pose Style': 'None', 'Radius': 0.03, 'Shaft Diameter': 0.1,
            'Shaft Length': 0.1, 'Topic': qos(topic, 5, transient), 'Value': True}


def grid_map(name, topic, scheme='costmap', alpha=0.5, transient=True, enabled=True):
    return {'Class': 'rviz_default_plugins/Map', 'Name': name, 'Enabled': enabled, 'Alpha': alpha,
            'Color Scheme': scheme, 'Draw Behind': False, 'Binary representation': False,
            'Binary threshold': 100, 'Topic': qos(topic, 1, transient),
            'Update Topic': qos(topic + '_updates', 1), 'Use Timestamp': False, 'Value': enabled}


def overlay(name, topic):
    return {'Class': 'rviz_2d_overlay_plugins/TextOverlay', 'Name': name, 'Enabled': True,
            'Topic': qos(topic, 2), 'Value': True, 'Overtake Position Properties': False,
            'Overtake FG Color Properties': False, 'Overtake BG Color Properties': False, 'Align Bottom': False}


def tf(enabled=False):
    return {'Class': 'rviz_default_plugins/TF', 'Name': 'TF', 'Enabled': enabled, 'Marker Scale': 3,
            'Show Arrows': False, 'Show Axes': True, 'Show Names': False, 'Update Interval': 0,
            'Frame Timeout': 15, 'Frames': {'All Enabled': True}, 'Tree': {}, 'Value': enabled}


def follow(target, distance, yaw, pitch, focal=(0, 0, 0)):
    return {'Class': 'rviz_default_plugins/ThirdPersonFollower', 'Distance': distance,
            'Focal Point': {'X': focal[0], 'Y': focal[1], 'Z': focal[2]}, 'Focal Shape Fixed Size': True,
            'Focal Shape Size': 0.01, 'Invert Z Axis': False, 'Name': 'Current View',
            'Near Clip Distance': 0.05, 'Pitch': pitch, 'Target Frame': target,
            'Value': 'ThirdPersonFollower (rviz_default_plugins)', 'Yaw': yaw}


def orbit(target, distance, yaw, pitch, focal=(0, 0, 0)):
    d = follow(target, distance, yaw, pitch, focal)
    d['Class'] = 'rviz_default_plugins/Orbit'
    d['Value'] = 'Orbit (rviz_default_plugins)'
    return d


def topdown(x, y, scale, angle=0.0, target='<Fixed Frame>'):
    return {'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Current View', 'Near Clip Distance': 0.01,
            'Scale': scale, 'Angle': angle, 'Target Frame': target, 'Value': 'TopDownOrtho (rviz_default_plugins)',
            'X': x, 'Y': y, 'Invert Z Axis': False}


def config(displays, view, bg=(30, 34, 42), width=WIN_W, x=0):
    return {'Panels': [],
            'Visualization Manager': {
                'Class': '', 'Displays': displays, 'Enabled': True,
                'Global Options': {'Background Color': rgb(bg), 'Fixed Frame': 'map', 'Frame Rate': 30},
                'Name': 'root', 'Tools': [], 'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
                'Value': True, 'Views': {'Current': view, 'Saved': None}},
            'Window Geometry': {'Height': WIN_H, 'Width': width, 'X': x, 'Y': 0,
                                'Hide Left Dock': True, 'Hide Right Dock': True}}


# Split screen: two 960x1080 panels side by side (window = panel + 50 px chrome).
HALF_W = 960 + 50


def ego():  # vehicle body box + LiDAR frame, published by iad_tools/vehicle_marker
    return [marker_array('Ego', '/viz/ego', depth=1)]


def pcd_map(alpha=0.35, size=0.25):
    return cloud('Point-cloud map', '/map/pointcloud_map', 'axis', size_m=size, alpha=alpha, transient=True,
                 best_effort=False, zmin=-3, zmax=12)


def perception_displays(ns='/perception'):
    return [cloud('Ground', ns + '/points/ground', 'flat', (90, 200, 120), size_m=0.10, alpha=0.75),
            cloud('Obstacles', ns + '/points/obstacle', 'flat', (255, 72, 56), size_m=0.15),
            cloud('Overhang', ns + '/points/overhang', 'flat', (160, 120, 255), size_m=0.12),
            cloud('Dust (removed)', ns + '/points/dust', 'flat', (255, 214, 64), size_m=0.12, alpha=0.8),
            grid_map('Occupancy', ns + '/occupancy_grid', 'costmap', alpha=0.3, transient=False),
            marker_array('Objects', ns + '/objects/markers', depth=5, transient=False)]


def localization_displays():
    return [pcd_map(),
            cloud('NDT aligned scan', '/localization/ndt/aligned_points', 'flat', (80, 220, 255), size_m=0.2),
            path('Dead reckoning', '/localization/debug/paths/dead_reckoning', (255, 120, 60), width=0.25),
            path('GNSS only (gated)', '/localization/debug/paths/gnss_gated', (255, 200, 40), width=0.25),
            path('Fused (EKF + NDT)', '/localization/debug/paths/fused', (90, 255, 140), width=0.35),
            marker_array('Covariance + GNSS fixes', '/localization/debug/markers', depth=2, transient=False)]


def planning_displays(search=True):
    d = [grid_map('Site costmap', '/map/occupancy_grid', 'map', alpha=0.55),
         grid_map('Live occupancy', '/perception/occupancy_grid', 'costmap', alpha=0.3, transient=False),
         marker_array('Trajectory', '/planning/markers', depth=1)]
    if search:
        d.append(marker_array('Search tree', '/planning/search', depth=1))
    return d


def control_displays():
    return [path('Reference', '/control/reference', (90, 170, 255), width=0.35, transient=True),
            path('MPC horizon', '/control/predicted', (120, 255, 120), width=0.5),
            path('Driven trail', '/control/trail', (255, 255, 255), width=0.3)]


def calibration_displays():
    cols = {'front_left': (255, 90, 90), 'front_right': (90, 255, 120), 'rear_left': (90, 160, 255), 'rear_right': (255, 210, 60)}
    return [cloud('Reference map (top LiDAR)', '/calibration/reference', 'flat', (200, 200, 200), size_m=0.12, alpha=0.5,
                  transient=True, best_effort=False)] + \
           [cloud(n, '/calibration/clouds/' + n, 'flat', c, size_m=0.14, best_effort=False) for n, c in cols.items()] + [tf(True)]


# (output path relative to src/, config)
CONFIGS = {
    # ---- standalone views, one per subsystem package
    'iad_perception/rviz/perception.rviz': config(
        perception_displays() + [cloud('Raw LiDAR', '/sensing/lidar/top/points', 'intensity', size_m=0.08, imax=100, enabled=False)],
        follow('base_link', 55, 3.55, 0.5, (8, 0, 1))),
    'iad_localization/rviz/localization.rviz': config(localization_displays() + [tf(False)], topdown(150, 100, 2.5)),
    'iad_planning/rviz/planning.rviz': config(planning_displays(), topdown(150, 100, 2.5)),
    'iad_control/rviz/control.rviz': config(control_displays(), topdown(30, 0, 9.0)),
    'iad_calibration/rviz/calibration.rviz': config(calibration_displays(), follow('base_link', 45, 3.4, 0.6, (5, 0, 1))),
    # ---- demo views (iad_bringup): + ego body, HUD, ground-truth trails from iad_eval
    'iad_bringup/rviz/perception_demo.rviz': config(
        ego() + [pcd_map(0.25)] + perception_displays() + [overlay('HUD', '/hud/perception')],
        follow('base_link', 52, 3.6, 0.5, (10, 0, 1))),
    'iad_bringup/rviz/perception_naive.rviz': config(
        ego() + [pcd_map(0.25)] + perception_displays('/perception_naive') + [overlay('HUD', '/hud/perception_naive')],
        follow('base_link', 52, 3.6, 0.5, (10, 0, 1)), width=HALF_W),
    'iad_bringup/rviz/perception_ours.rviz': config(
        ego() + [pcd_map(0.25)] + perception_displays() + [overlay('HUD', '/hud/perception')],
        follow('base_link', 52, 3.6, 0.5, (10, 0, 1)), width=HALF_W, x=HALF_W),
    'iad_bringup/rviz/localization_demo.rviz': config(
        ego() + localization_displays() +
        [path('Ground truth', '/eval/localization/truth_path', (255, 255, 255), width=0.2), overlay('HUD', '/hud/localization')],
        follow('base_link', 75, 3.9, 0.95, (0, 0, 0))),
    'iad_bringup/rviz/planning_demo.rviz': config(
        ego() + [pcd_map(0.45, 0.6), grid_map('Site costmap', '/map/occupancy_grid', 'costmap', alpha=0.35),
                 grid_map('Live occupancy', '/perception/occupancy_grid', 'costmap', alpha=0.5, transient=False),
                 marker_array('Trajectory', '/planning/markers', depth=1), marker_array('Search tree', '/planning/search', depth=1)] +
        [path('Driven trail', '/control/trail', (255, 255, 255), width=0.8), overlay('HUD', '/hud/planning')],
        topdown(168, 103, 4.4)),
    'iad_bringup/rviz/control_demo.rviz': config(
        ego() + [pcd_map(0.3), grid_map('Site costmap', '/map/occupancy_grid', 'map', alpha=0.35)] + control_displays() +
        [marker_array('Trajectory', '/planning/markers', depth=1),
                                                       overlay('HUD', '/hud/control')],
        follow('base_link', 48, 3.3, 0.75, (6, 0, 0))),
    'iad_bringup/rviz/calibration_demo.rviz': config(
        ego() + [pcd_map(0.2)] + calibration_displays() + [overlay('HUD', '/hud/calibration')], follow('base_link', 42, 3.5, 0.6, (5, 0, 1))),
    'iad_bringup/rviz/mission_demo.rviz': config(
        ego() + [pcd_map(0.3)] + perception_displays()[1:] + [marker_array('Trajectory', '/planning/markers', depth=1)] +
        control_displays()[1:] + [overlay('HUD', '/hud/mission')],
        follow('base_link', 60, 3.6, 0.55, (10, 0, 1))),
}


def main(names=None):
    for name, cfg in CONFIGS.items():
        if names and not any(n in name for n in names):
            continue
        out = os.path.join(SRC, name)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False, width=120)
        print('wrote', name)


if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
