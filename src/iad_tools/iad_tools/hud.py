"""Text HUD overlays for RViz (rviz_2d_overlay_plugins), rendered from the JSON metric topics.

One node per demo: `profile` selects which topics are read and which panels are drawn.
"""
import json
import math
import re

import rclpy
from rclpy.node import Node
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA, String

GOOD, BAD, WARN, DIM, ACC, FG = '#7CF29C', '#FF6B6B', '#FFC857', '#9AA4B2', '#6EC1FF', '#ECEFF3'


def span(text, color, bold=False):
    t = f'<b>{text}</b>' if bold else text
    return f'<span style="color:{color};">{t}</span>'


def pct(x):
    return f'{100.0 * x:5.1f} %'


class Hud(Node):
    VEH = '/sim/vehicle/debug'
    SOURCES = {
        'perception': ['/perception/metrics', '/perception_naive/metrics', '/eval/perception', '/eval/perception_naive', VEH],
        'localization': ['/localization/metrics', '/eval/localization', '/sim/gnss/debug', VEH],
        'planning': ['/planning/metrics', '/control/metrics', '/mission/status', VEH],
        'control': ['/control/metrics', '/eval/control', VEH],
        'calibration': ['/calibration/metrics', '/eval/calibration', VEH],
        'mission': ['/perception/metrics', '/eval/perception', '/localization/metrics', '/eval/localization', '/planning/metrics',
                    '/control/metrics', '/eval/control', '/sim/gnss/debug', '/mission/status', VEH],
    }
    SURFACES = ['asphalt', 'gravel', 'dirt', 'mud', 'rock', 'concrete', 'water']

    def __init__(self):
        super().__init__('hud')
        self.profile = self.declare_parameter('profile', 'perception').value
        self.title = self.declare_parameter('title', '').value
        self.data = {}
        for t in self.SOURCES.get(self.profile, []):
            self.create_subscription(String, t, lambda m, t=t: self.on(t, m), 10)
        self.pubs = {}
        self.create_timer(0.2, self.tick)

    def on(self, topic, msg):
        try:
            self.data[topic] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def d(self, topic):
        return self.data.get(topic, {})

    def panel(self, topic, html, x=18, y=18, w=560, h=200, size=12.0, align=OverlayText.LEFT,
              valign=OverlayText.TOP, bg=(0.06, 0.07, 0.09, 0.72)):
        if topic not in self.pubs:
            self.pubs[topic] = self.create_publisher(OverlayText, topic, 2)
        m = OverlayText()
        m.action = OverlayText.ADD
        m.width, m.height = int(w), int(h)
        m.horizontal_distance, m.vertical_distance = int(x), int(y)
        m.horizontal_alignment, m.vertical_alignment = align, valign
        m.bg_color = ColorRGBA(r=bg[0], g=bg[1], b=bg[2], a=bg[3])
        m.fg_color = ColorRGBA(r=0.93, g=0.95, b=0.97, a=1.0)
        m.line_width, m.text_size, m.font = 2, float(size), 'DejaVu Sans Mono'
        # Qt rich text collapses runs of spaces: keep column alignment with &nbsp; outside the tags.
        m.text = ''.join(p if p.startswith('<') else p.replace(' ', '&nbsp;') for p in re.split(r'(<[^>]+>)', html))
        self.pubs[topic].publish(m)

    def vehicle_line(self):
        s = self.d(self.VEH)
        if not s:
            return span('waiting for Gazebo...', DIM)
        grade = -100.0 * math.tan(s.get('pitch', 0.0))
        surf = self.SURFACES[s['surface']] if 0 <= s.get('surface', -1) < len(self.SURFACES) else ''
        return span(f"sim t {s.get('t', 0):5.1f} s   {3.6 * abs(s.get('v', 0)):4.1f} km/h   grade {grade:+4.1f} %   {surf}", DIM)

    def tick(self):
        getattr(self, 'render_' + self.profile, lambda: None)()

    # ------------------------------------------------------------------ perception (split screen)
    def render_perception(self):
        for side, topic, ev, title, sub in (
                ('perception_naive', '/perception_naive/metrics', '/eval/perception_naive', 'BASELINE',
                 'one RANSAC ground plane, no dust filter'),
                ('perception', '/perception/metrics', '/eval/perception', 'THIS STACK',
                 'range-image + temporal dust filter, CZM ground')):
            m = dict(self.d(ev), median_ms=self.d(topic).get('median_ms', 0))
            ours = side == 'perception'
            fo, ph = m.get('false_obstacle_pts', 0), m.get('phantom_cells', 0)
            gr = m.get('ground_recall', 0)
            lines = [span(title, ACC if ours else WARN, True),
                     span(sub, DIM),
                     f"false obstacle points  {span(f'{fo:6d}', GOOD if fo < 150 else BAD)} /frame",
                     f"phantom occupied cells {span(f'{ph:6d}', GOOD if ph < 60 else BAD)}",
                     f"dust echoes removed    {span(pct(m.get('dust_recall', 0)), GOOD if ours else BAD)}",
                     f"ground kept as ground  {span(pct(gr), GOOD if gr > .9 else BAD)}",
                     f"latency (median)       {m.get('median_ms', 0):5.1f} ms",
                     self.vehicle_line()]
            self.panel('/hud/' + side, '<br>'.join(lines), w=520, h=186, size=12.5)


    # ------------------------------------------------------------------ localization
    def render_localization(self):
        m, ev, g = self.d('/localization/metrics'), self.d('/eval/localization'), self.d('/sim/gnss/debug')
        st = m.get('gnss_status', '...')
        stc = GOOD if st == 'RTK_FIX' else WARN if st == 'RTK_FLOAT' else BAD
        ge = g.get('error', 0.0)
        gec = BAD if ge > 1 else WARN if ge > 0.2 else GOOD
        lines = [span('LOCALIZATION', ACC, True) + '  ' + span('container terminal, GNSS multipath + crane outage', DIM),
                 f"GNSS {span(f'{st:9s}', stc, True)}  sky blocked {100 * g.get('sky_blocked', 0):3.0f} %   "
                 f"fix error {span(f'{ge:5.2f} m', gec)}",
                 span('position error          now     RMS     max', DIM)]
        for key, label, col in (('fused', 'EKF + NDT (this stack)', GOOD), ('gnss_gated', 'EKF + gated GNSS', '#FFAA28'),
                                ('gnss_raw', 'EKF + raw GNSS', '#FF5A50'), ('dead_reckoning', 'dead reckoning', '#BE78FF')):
            e = ev.get(key, {})
            lines.append(span(f"{label:22s}", col) + f" {e.get('err', 0):6.2f}  {e.get('rms', 0):6.2f}  {e.get('max', 0):6.2f} m")
        lines.append(span(f"NDT {m.get('ndt_iter', 0):2d} it  {m.get('ndt_ms', 0):4.1f} ms  fit {m.get('ndt_nvtl', 0):.2f}  "
                          f"sigma {100 * m.get('ndt_std', 0):3.0f} cm   GNSS gated out {m.get('gnss_rejected_fused', 0)}", DIM))
        lines.append(self.vehicle_line())
        self.panel('/hud/localization', '<br>'.join(lines), w=700, h=214, size=12.5)

    # ------------------------------------------------------------------ control
    def render_control(self):
        c, ev = self.d('/control/metrics'), self.d('/eval/control')
        mpc = c.get('algorithm') == 'mpc'
        title = (span('THIS STACK', ACC, True) + '  ' + span('MPC + dead-time comp. + grade FF', DIM)) if mpc else \
                (span('BASELINE', WARN, True) + '  ' + span('pure pursuit, no compensation', DIM))
        lat, mx = abs(ev.get('lat_err', 0)), ev.get('lat_max', 0)
        lines = [title,
                 f"lateral error   {span(f'{lat:5.2f} m', GOOD if lat < 0.25 else BAD)}   max {span(f'{mx:5.2f} m', GOOD if mx < 0.6 else BAD)}",
                 f"heading error   {ev.get('head_err_deg', 0):+5.1f} deg   RMS {ev.get('head_rms_deg', 0):4.1f} deg",
                 f"speed {3.6 * abs(c.get('v', 0)):4.1f} km/h  target {3.6 * c.get('v_des', 0):4.1f}   gear {c.get('state', '')}",
                 f"steer cmd {c.get('steer_cmd', 0):+5.2f}  actual {c.get('steer', 0):+5.2f} rad"]
        if ev.get('docked'):
            dl, dh = math.hypot(ev.get('dock_long', 0), ev.get('dock_lat', 0)), abs(ev.get('dock_head_deg', 0))
            lines.append(f"DOCKED  position {span(f'{dl:4.2f} m', GOOD if dl < 0.5 else BAD)}  heading {span(f'{dh:3.1f} deg', GOOD if dh < 3 else BAD)}")
        else:
            lines.append(self.vehicle_line())
        self.panel('/hud/control', '<br>'.join(lines), w=520, h=162, size=12.5)

    # ------------------------------------------------------------------ planning
    def render_planning(self):
        p, ms = self.d('/planning/metrics'), self.d('/mission/status')
        st = p.get('state', '...')
        stc = GOOD if st in ('driving', 'arrived') else WARN if st == 'stopped_for_obstacle' else BAD if st == 'failed' else DIM
        goal = ms.get('goal', '')
        lines = [span('PLANNING', ACC, True) + '  ' + span('no lanes: Hybrid A* + Reeds-Shepp', DIM),
                 f"goal {span(goal or '-', FG, True)}   ({ms.get('index', 0) + 1}/{ms.get('count', 0)})   state {span(st, stc, True)}",
                 f"plan {p.get('plan_ms', 0):5.0f} ms   {p.get('expansions', 0):6d} nodes   RS shot {'yes' if p.get('rs_shot') else 'no'}",
                 f"path {p.get('length', 0):5.0f} m   cusps {p.get('cusps', 0)}   remaining {p.get('remaining', 0):5.0f} m   replans {p.get('replans', 0)}"]
        if st == 'stopped_for_obstacle':
            lines.append(span(f"unmapped obstacle {p.get('blocked_at', 0):4.1f} m ahead: holding, replan if it stays", WARN))
        lines.append(self.vehicle_line())
        self.panel('/hud/planning', '<br>'.join(lines), w=600, h=146, size=12.5)


    # ------------------------------------------------------------------ calibration
    def render_calibration(self):
        m, ev = self.d('/calibration/metrics'), self.d('/eval/calibration')
        st = m.get('state', '...')
        step = {'ground': 'ground plane (parked): roll, pitch, height',
                'collect': f"driving: keyframes {m.get('keyframes', 0)}/{m.get('max_keyframes', 0)} + roof-LiDAR reference map",
                'register': f"joint registration: iteration {m.get('iteration', 0)}/{m.get('max_iterations', 0)}  (yaw, x, y)",
                'done': 'done: extrinsics estimated'}.get(st, st)
        lines = [span('CALIBRATION', ACC, True) + '  ' + span('4 corner LiDARs vs the roof LiDAR, target-less', DIM),
                 span(step, GOOD if st == 'done' else WARN, True),
                 span('sensor         rotation error      translation error', DIM)]
        for n, e in sorted(ev.get('lidars', {}).items()):
            r0, r1, t0, t1 = e.get('rot_before_deg', 0), e.get('rot_deg', 0), e.get('trans_before_cm', 0), e.get('trans_cm', 0)
            lines.append(f"{n:12s} {r0:4.2f} -> {span(f'{r1:4.2f} deg', GOOD if r1 < 0.3 else WARN)}   "
                         f"{t0:4.1f} -> {span(f'{t1:4.1f} cm', GOOD if t1 < 5 else WARN)}")
        lines.append(self.vehicle_line())
        self.panel('/hud/calibration', '<br>'.join(lines), w=640, h=178, size=12.5)


    # ------------------------------------------------------------------ full mission
    def render_mission(self):
        ms, pe, lo = self.d('/mission/status'), dict(self.d('/eval/perception'), **self.d('/perception/metrics')), self.d('/localization/metrics')
        pl, co, sd = self.d('/planning/metrics'), dict(self.d('/control/metrics'), **self.d('/eval/control')), self.d(self.VEH)
        g = {'status': lo.get('gnss_status', '-')}
        le = self.d('/eval/localization').get('fused', {}).get('err', 0.0)
        mass = sd.get('mass', 0) / 1000.0
        st = pl.get('state', '...')
        lines = [span('FULL STACK', ACC, True) + '  ' + span('quarry haul cycle: load, haul, dump', DIM),
                 f"goal {span(ms.get('goal') or '-', FG, True)} ({ms.get('index', 0) + 1}/{ms.get('count', 0)})   "
                 f"{span(st, WARN if st == 'stopped_for_obstacle' else GOOD)}   mass {mass:3.0f} t",
                 f"perception    dust removed {pct(pe.get('dust_recall', 0))}   {pe.get('objects', 0):2d} objects   {pe.get('median_ms', 0):4.1f} ms",
                 f"localization  error {span(f'{le:4.2f} m', GOOD if le < 0.2 else WARN)}   GNSS {g.get('status', '-'):9s}  NDT fit {lo.get('ndt_nvtl', 0):.2f}",
                 f"planning      path {pl.get('length', 0):4.0f} m   cusps {pl.get('cusps', 0)}   replans {pl.get('replans', 0)}",
                 f"control       lateral {abs(co.get('lat_err', 0)):4.2f} m   heading {co.get('head_err_deg', 0):+5.1f} deg   {co.get('state', '')}",
                 self.vehicle_line()]
        self.panel('/hud/mission', '<br>'.join(lines), w=640, h=172, size=12.5)


def main():
    rclpy.init()
    rclpy.spin(Hud())


if __name__ == '__main__':
    main()
