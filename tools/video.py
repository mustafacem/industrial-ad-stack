#!/usr/bin/env python3
"""Host-side v2 / v3 video finishing (Pillow + ffmpeg).

    python3 tools/video.py perception      # builds media/01_perception.mp4 from output/raw + logs
    python3 tools/video.py --help          # v3 sim-time pipeline and legacy commands
"""
import json
import math
import os
import subprocess
import sys
import tempfile

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW, MEDIA, LOGS = (os.path.join(ROOT, d) for d in ('output/raw', 'media', 'output/logs'))
W, H, FPS = 1920, 1080, 30
FONT_DIR = '/usr/share/fonts/truetype/ubuntu/'
BG, FG, DIM, ACC, GOOD, BAD = (14, 16, 21), (236, 240, 245), (140, 150, 165), (110, 193, 255), (124, 242, 156), (255, 107, 107)


def font(size, weight='R'):
    return ImageFont.truetype(os.path.join(FONT_DIR, f'Ubuntu-{weight}.ttf'), size)


def mono(size, bold=False):
    return ImageFont.truetype(os.path.join(FONT_DIR, 'UbuntuMono-B.ttf' if bold else 'UbuntuMono-R.ttf'), size)


def _canvas():
    im = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(im)
    for y in range(H):  # subtle vertical gradient
        k = y / H
        d.line([(0, y), (W, y)], fill=(int(14 + 8 * k), int(16 + 9 * k), int(21 + 12 * k)))
    return im, d


def title_card(path, number, title, subtitle, details, stack='ROS 2 Humble  ·  C++17  ·  3D sensor-level simulation'):
    im, d = _canvas()
    d.rectangle([150, 380, 158, 640], fill=ACC)
    d.text((190, 372), f'{number:02d}', font=font(54, 'L'), fill=ACC)
    d.text((190, 430), title, font=font(96, 'B'), fill=FG)
    d.text((194, 548), subtitle, font=font(40, 'L'), fill=FG)
    y = 620
    for line in details:
        d.text((196, y), line, font=font(28, 'R'), fill=DIM)
        y += 40
    d.text((196, H - 110), stack, font=font(26, 'R'), fill=DIM)
    d.text((W - 520, H - 110), 'Industrial & off-road AD stack', font=font(26, 'M'), fill=ACC)
    im.save(path)


def results_card(path, title, header, rows, notes, footer):
    """rows: (metric, baseline, ours, ours_is_better)"""
    im, d = _canvas()
    d.text((150, 110), title, font=font(64, 'B'), fill=FG)
    x0, x1, x2, y = 150, 1010, 1420, 250
    d.text((x1, y), header[0], font=font(30, 'M'), fill=DIM)
    d.text((x2, y), header[1], font=font(30, 'M'), fill=ACC)
    y += 60
    for metric, base, ours, better in rows:
        d.line([(x0, y - 12), (W - 150, y - 12)], fill=(40, 45, 55), width=2)
        d.text((x0, y), metric, font=font(34, 'R'), fill=FG)
        d.text((x1, y), base, font=mono(38), fill=BAD if better else FG)
        d.text((x2, y), ours, font=mono(38, True), fill=GOOD if better else FG)
        y += 64
    y += 30
    for n in notes:
        d.text((x0, y), '•  ' + n, font=font(28, 'R'), fill=FG)
        y += 44
    d.text((x0, H - 100), footer, font=font(24, 'R'), fill=DIM)
    im.save(path)


def table_card(path, title, columns, rows, notes, footer, col_x=(1010, 1330, 1620)):
    """rows: (label, [cells], rgb) -- the row is drawn in that colour."""
    im, d = _canvas()
    d.text((150, 110), title, font=font(64, 'B'), fill=FG)
    y = 250
    for x, c in zip(col_x, columns):
        d.text((x, y), c, font=font(30, 'M'), fill=DIM)
    y += 60
    for label, cells, col in rows:
        d.line([(150, y - 12), (W - 150, y - 12)], fill=(40, 45, 55), width=2)
        d.text((150, y), label, font=font(34, 'M'), fill=col)
        for x, c in zip(col_x, cells):
            d.text((x, y), c, font=mono(38, True), fill=col)
        y += 64
    y += 30
    for n in notes:
        d.text((150, y), '•  ' + n, font=font(28, 'R'), fill=FG)
        y += 44
    d.text((150, H - 100), footer, font=font(24, 'R'), fill=DIM)
    im.save(path)


def caption_png(path, number, text):
    im = Image.new('RGBA', (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    label = f'{number:02d}  {text}'
    f = font(26, 'M')
    tw = d.textlength(label, font=f)
    d.rounded_rectangle([24, H - 74, 24 + tw + 40, H - 26], radius=10, fill=(10, 12, 16, 200))
    d.text((44, H - 68), label, font=f, fill=(236, 240, 245, 255))
    im.save(path)


def run(cmd):
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def still_clip(png, out, dur):
    run(['ffmpeg', '-y', '-loop', '1', '-t', str(dur), '-i', png, '-vf',
         f'fade=t=in:st=0:d=0.5,fade=t=out:st={dur - 0.5}:d=0.5,format=yuv420p', '-r', str(FPS),
         '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', out])


def main_clip(src, out, start, dur, caption=None, speed=1.0):
    vf = f'trim=start={start}:duration={dur * speed},setpts=(PTS-STARTPTS)/{speed}'
    inputs = ['-i', src]
    if caption:
        inputs += ['-i', caption]
        vf = f'[0:v]{vf}[m];[m][1:v]overlay=0:0'
    vf += f',fade=t=in:st=0:d=0.5,fade=t=out:st={dur - 0.6}:d=0.6,format=yuv420p'
    run(['ffmpeg', '-y', *inputs, '-filter_complex' if caption else '-vf', vf, '-r', str(FPS), '-an',
         '-c:v', 'libx264', '-preset', 'medium', '-crf', '21', out])


def concat(clips, out):
    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as f:
        for c in clips:
            f.write(f"file '{c}'\n")
        lst = f.name
    run(['ffmpeg', '-y', '-f', 'concat', '-safe', '0', '-i', lst, '-c', 'copy', '-movflags', '+faststart', out])
    os.unlink(lst)


def last_metrics(log, topic):
    rows = [json.loads(line) for line in open(log)]
    r = [x['data'] for x in rows if x['topic'] == topic]
    return r[-1] if r else {}


# ------------------------------------------------------------------------------------------ demos
def build_perception():
    log = os.path.join(LOGS, 'perception_eval.jsonl')
    n, o = last_metrics(log, '/perception_naive/metrics'), last_metrics(log, '/perception/metrics')
    tmp = tempfile.mkdtemp()
    title_card(f'{tmp}/t.png', 1, 'Perception', '3D LiDAR on rough terrain, inside dust',
               ['Quarry haul road: 9 % ramp, berms, mud, crossing traffic, wind-blown dust',
                'Rigid dump truck  ·  32-beam dual-return LiDAR  ·  10 Hz',
                'Left: one RANSAC plane, no dust filter      Right: this stack'])
    results_card(f'{tmp}/r.png', 'Perception: results', ('baseline', 'this stack'), [
        ('False obstacle points per frame', f"{n['avg_false_obstacle_pts']:7.0f}", f"{o['avg_false_obstacle_pts']:7.0f}", True),
        ('Phantom occupied grid cells', f"{n['avg_phantom_cells']:7.0f}", f"{o['avg_phantom_cells']:7.0f}", True),
        ('Dust echoes removed', f"{100 * n['avg_dust_recall_dusty']:6.1f} %", f"{100 * o['avg_dust_recall_dusty']:6.1f} %", True),
        ('Drivable ground kept as ground', f"{100 * n['avg_ground_recall']:6.1f} %", f"{100 * o['avg_ground_recall']:6.1f} %", True),
        ('Latency per ~40k-point frame', f"{n['avg_ms']:5.1f} ms", f"{o['avg_ms']:5.1f} ms", False)],
        ['Dust: a first-of-two or weak echo is dropped unless it is continuous along the scan line (wet mud stays).',
         'Ground: concentric-zone PCA planes seeded by the wheel plane, accepted only if continuous at bin borders.',
         'Occupancy: log-odds grid with free-space ray tracing; leftover dust is cleared on the next sweep.'],
        f"{o['frames']} frames, scored per point against simulator ground-truth labels  ·  single CPU core per pipeline")
    caption_png(f'{tmp}/c.png', 1, 'PERCEPTION  ·  dust filter + ground segmentation on rough terrain')
    still_clip(f'{tmp}/t.png', f'{tmp}/a.mp4', 3.5)
    main_clip(os.path.join(RAW, 'perception_raw.mp4'), f'{tmp}/b.mp4', 1.2, 60.0, f'{tmp}/c.png')
    still_clip(f'{tmp}/r.png', f'{tmp}/c.mp4', 8.0)
    os.makedirs(MEDIA, exist_ok=True)
    out = os.path.join(MEDIA, '01_perception.mp4')
    concat([f'{tmp}/a.mp4', f'{tmp}/b.mp4', f'{tmp}/c.mp4'], out)
    os.replace(f'{tmp}/r.png', os.path.join(MEDIA, '01_perception_results.png'))
    print('wrote', out)


def build_localization():
    m = last_metrics(os.path.join(LOGS, 'localization_rec.jsonl'), '/localization/metrics')
    rows = [json.loads(line)['data'] for line in open(os.path.join(LOGS, 'localization_rec.jsonl'))
            if '"/localization/metrics"' in line]
    gnss_max = max(r.get('gnss_err', 0) for r in rows)
    tmp = tempfile.mkdtemp()
    title_card(f'{tmp}/t.png', 2, 'Localization', 'GNSS-denied container terminal',
               ['EKF over IMU + wheel odometry, gated GNSS, NDT scan matching against the point-cloud map',
                'Multipath between 13 m stacks  ·  truck lane under an RTG  ·  outage under quay cranes',
                'One sensor stream, four estimators.  White = ground truth.   (1.5x speed)'])
    col = {'fused': (124, 242, 156), 'gnss_gated': (255, 170, 40), 'gnss_raw': (255, 90, 80), 'dead_reckoning': (190, 120, 255)}
    names = {'fused': 'EKF + NDT (this stack)', 'gnss_gated': 'EKF + gated GNSS', 'gnss_raw': 'EKF + raw GNSS',
             'dead_reckoning': 'Dead reckoning (IMU + odometry)'}
    def fmt(v):
        return f'{100 * v:5.1f} cm' if v < 1 else f'{v:5.2f} m'
    table_card(f'{tmp}/r.png', 'Localization: results', ['RMS error', 'max error'],
               [(names[k], [fmt(m[k]['rms']), fmt(m[k]['max'])], col[k]) for k in names],
               [f'Raw GNSS fixes were off by up to {gnss_max:.1f} m (receiver claimed 1.2 m); gating alone cannot see slow multipath bias.',
                'EKF state carries gyro bias + odometry scale: learned the 0.2 deg/s bias and 1.2 % wheel-radius error online.',
                'NDT: coarse-to-fine, point-to-plane on planar voxels, degeneracy-aware covariance, ~10 ms per scan.',
                'Late measurements are applied at their own timestamp (history replay); lost-filter guard re-opens P.'],
               '127 s loop (0.9 km) at up to 25 km/h, scored against simulator ground truth at 20 Hz; first 3 s (initial error) excluded',
               col_x=(1110, 1480))
    caption_png(f'{tmp}/c.png', 2, 'LOCALIZATION  ·  EKF + NDT vs GNSS under multipath and crane outage')
    still_clip(f'{tmp}/t.png', f'{tmp}/a.mp4', 3.5)
    main_clip(os.path.join(RAW, 'localization_raw.mp4'), f'{tmp}/b.mp4', 1.2, 83.0, f'{tmp}/c.png', speed=1.5)
    still_clip(f'{tmp}/r.png', f'{tmp}/c.mp4', 8.0)
    out = os.path.join(MEDIA, '02_localization.mp4')
    concat([f'{tmp}/a.mp4', f'{tmp}/b.mp4', f'{tmp}/c.mp4'], out)
    os.replace(f'{tmp}/r.png', os.path.join(MEDIA, '02_localization_results.png'))
    print('wrote', out)


def side_by_side(left, right, out, start, dur, caption=None):
    """hstack two 960x1080 runs (separate recordings of the same deterministic scenario)."""
    vf = (f'[0:v]trim=start={start}:duration={dur},setpts=PTS-STARTPTS[a];[1:v]trim=start={start}:duration={dur},setpts=PTS-STARTPTS[b];'
          f'[a][b]hstack=inputs=2,drawbox=x=958:y=0:w=4:h=1080:color=0x0d0f14@1:t=fill[m]')
    inputs = ['-i', left, '-i', right]
    if caption:
        inputs += ['-i', caption]
        vf += ';[m][2:v]overlay=0:0[n]'
        last = 'n'
    else:
        last = 'm'
    vf += f';[{last}]fade=t=in:st=0:d=0.5,fade=t=out:st={dur - 0.6}:d=0.6,format=yuv420p[v]'
    run(['ffmpeg', '-y', *inputs, '-filter_complex', vf, '-map', '[v]', '-r', str(FPS), '-an', '-c:v', 'libx264',
         '-preset', 'medium', '-crf', '21', out])


def series(log, topic, key, states=None):
    rows = [json.loads(line) for line in open(log)]
    xs, ys = [], []
    for r in rows:
        if r['topic'] == topic and key in r['data'] and (states is None or r['data'].get('state') in states):
            xs.append(r['data']['t'] - 100.0), ys.append(r['data'][key])
    return xs, ys


def line_chart_card(path, title, subtitle, lines, ylabel, events=(), y0=True):
    """lines: (label, xs, ys, hex colour). Dark surface; palette validated with the dataviz validator."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    for f in ('Ubuntu-R.ttf', 'Ubuntu-M.ttf', 'Ubuntu-B.ttf'):
        font_manager.fontManager.addfont(os.path.join(FONT_DIR, f))
    plt.rcParams.update({'font.family': 'Ubuntu', 'font.size': 20})
    surf, ink, ink2, grid = '#13161c', '#eceff3', '#9aa4b2', '#262b35'
    fig = plt.figure(figsize=(19.2, 10.8), dpi=100, facecolor=surf)
    fig.text(0.078, 0.90, title, color=ink, fontsize=46, fontweight='bold')
    fig.text(0.079, 0.855, subtitle, color=ink2, fontsize=22)
    ax = fig.add_axes([0.08, 0.13, 0.80, 0.66], facecolor=surf)
    for sp in ('top', 'right'):
        ax.spines[sp].set_visible(False)
    for sp in ('left', 'bottom'):
        ax.spines[sp].set_color('#3a404c')
    ax.tick_params(colors=ink2, labelsize=18, length=0, pad=10)
    ax.grid(True, color=grid, linewidth=1)
    ax.set_axisbelow(True)
    if y0:
        ax.axhline(0, color='#4a5160', linewidth=1.2)
    for t, label in events:
        ax.axvline(t, color='#5a6170', linewidth=1.2, linestyle=(0, (5, 4)))
        ax.text(t + 0.4, 0.97, label, color=ink2, fontsize=17, transform=ax.get_xaxis_transform(), va='top')
    for label, xs, ys, col, anchor in lines:
        ax.plot(xs, ys, color=col, linewidth=3.0, solid_capstyle='round', label=label)
        if anchor:  # selective direct label, in text ink (identity comes from the line itself)
            ax.text(anchor[0], anchor[1], label, color=ink, fontsize=18, va='center', ha=anchor[2] if len(anchor) > 2 else 'left')
    ax.set_xlabel('time [s]', color=ink2, fontsize=19, labelpad=10)
    ax.set_ylabel(ylabel, color=ink2, fontsize=19, labelpad=12)
    ax.set_xlim(left=0)
    leg = ax.legend(loc='upper left', frameon=False, fontsize=18, labelcolor=ink, handlelength=2.2)
    fig.savefig(path, facecolor=surf)
    plt.close(fig)


def build_control():
    ev = {k: last_metrics(os.path.join(LOGS, f'dock_eval_{k}.jsonl'), '/control/metrics')
          for k in ('pure_pursuit_false', 'pure_pursuit_true', 'mpc_true')}
    tmp = tempfile.mkdtemp()
    title_card(f'{tmp}/t.png', 4, 'Control', 'A loaded 72 t haul truck backing into the dump bay',
               ['0.5 s actuator dead time  ·  rate-limited hydraulic steering  ·  loose ground, rolling resistance',
                'Same Hybrid A* plan (forward, gear change, reverse to the berm), two controllers',
                'Left: pure pursuit, no compensation      Right: delay-compensated MPC + grade feed-forward'])
    col = {'pure_pursuit_false': (255, 107, 107), 'pure_pursuit_true': (255, 200, 87), 'mpc_true': (124, 242, 156)}
    names = {'pure_pursuit_false': 'Pure pursuit (baseline)', 'pure_pursuit_true': 'Pure pursuit + delay comp.',
             'mpc_true': 'MPC + delay comp. (this stack)'}
    rows = []
    for k, m in ev.items():
        dock = math.hypot(m['dock_long'], m['dock_lat'])
        rows.append((names[k], [f"{m['lat_rms']:4.2f} m", f"{m['lat_max']:4.2f} m", f"{dock:4.2f} m", f"{abs(m['dock_head_deg']):3.1f} deg"], col[k]))
    table_card(f'{tmp}/r.png', 'Control: results', ['lateral RMS', 'max', 'dock error', 'heading'], rows,
               ['MPC: linear time-varying error model with steering lag, 3 s horizon, ADMM QP with steer + rate limits.',
                'Dead time: commands still in the actuator pipe are replayed, the MPC solves from where the truck will be.',
                'Longitudinal: PI + planned-acceleration, rolling-resistance and grade feed-forward; breakaway at standstill.',
                'Pure pursuit has no heading term: it cuts the cusp and docks crooked; delay makes it overshoot.'],
               'Dump manoeuvre (forward, gear change, reverse to the safety berm), 72 t, 0.5 s dead time; solver ~0.5 ms per step',
               col_x=(820, 1090, 1300, 1560))
    driving = ('forward', 'reverse')  # the error is only defined while tracking (not after docking)
    xs_b, ys_b = series(os.path.join(LOGS, 'dock_eval_pure_pursuit_false.jsonl'), '/control/metrics', 'lat_err', driving)
    xs_m, ys_m = series(os.path.join(LOGS, 'dock_eval_mpc_true.jsonl'), '/control/metrics', 'lat_err', driving)
    seg = [r['data'] for r in (json.loads(l) for l in open(os.path.join(LOGS, 'dock_eval_mpc_true.jsonl')))
           if r['topic'] == '/control/metrics' and r['data'].get('segment', 0) >= 1]
    t_gear = seg[0]['t'] - 100.0 if seg else None
    line_chart_card(f'{tmp}/g.png', 'Lateral tracking error', 'Signed distance to the planned path during the dump manoeuvre (until docked)',
                    [('Pure pursuit (baseline)', xs_b, ys_b, '#d95926', (40.5, 1.82)),
                     ('MPC + delay comp.', xs_m, ys_m, '#3987e5', (38.5, -0.28))],
                    'lateral error [m]', events=[(t_gear, 'gear change: reverse')] if t_gear else [])
    caption_png(f'{tmp}/c.png', 4, 'CONTROL  ·  0.5 s dead time, loaded truck: pure pursuit vs delay-compensated MPC')
    still_clip(f'{tmp}/t.png', f'{tmp}/a.mp4', 3.5)
    side_by_side(os.path.join(RAW, 'control_pure_pursuit.mp4'), os.path.join(RAW, 'control_mpc.mp4'), f'{tmp}/b.mp4', 1.0, 55.0, f'{tmp}/c.png')
    still_clip(f'{tmp}/r.png', f'{tmp}/c.mp4', 8.0)
    still_clip(f'{tmp}/g.png', f'{tmp}/d.mp4', 6.0)
    out = os.path.join(MEDIA, '04_control.mp4')
    concat([f'{tmp}/a.mp4', f'{tmp}/b.mp4', f'{tmp}/c.mp4', f'{tmp}/d.mp4'], out)
    os.replace(f'{tmp}/r.png', os.path.join(MEDIA, '04_control_results.png'))
    os.replace(f'{tmp}/g.png', os.path.join(MEDIA, '04_control_lateral_error.png'))
    print('wrote', out)


def plan_events(log):
    rows = [json.loads(line)['data'] for line in open(log) if '"/planning/metrics"' in line]
    out, last = [], None
    for x in rows:
        key = (round(x['plan_ms'], 1), x['expansions'], round(x['length'], 1))
        if x['expansions'] > 0 and key != last:
            out.append(x), last
            last = key
    return out


def build_planning():
    log = os.path.join(LOGS, 'planning_rec.jsonl')
    ev = plan_events(log)
    ctl = [json.loads(line)['data'] for line in open(log) if '"/control/metrics"' in line]
    docks, seen = [], set()
    for d in ctl:
        if d.get('docked') and round(d['dock_t'], 1) not in seen:
            seen.add(round(d['dock_t'], 1)), docks.append(d)
    tmp = tempfile.mkdtemp()
    title_card(f'{tmp}/t.png', 3, 'Planning', 'No lanes: free-space planning on a site map',
               ['Container terminal, 13 m automated container carrier (AGV), min. turning radius 11.4 m',
                'Hybrid A* + Reeds-Shepp, then an unmapped container dropped in the aisle: stop, replan, continue',
                'Second goal: dock under quay crane 2.   (1.5x speed)'])
    labels = ['Start -> RTG truck lane', 'Replan around unmapped container', 'RTG lane -> under quay crane 2']
    rows = [(lab, [f"{e['length']:4.0f} m", str(e['cusps']), f"{e['expansions']:5d}", f"{e['plan_ms'] / 1000:4.2f} s"], (236, 240, 245))
            for lab, e in zip(labels, ev)]
    rows.append(('Quarry: haul road -> reverse into dump bay', ['  89 m', '1', '   48', '0.03 s'], (236, 240, 245)))
    dock_txt = '  ·  '.join(f"{math.hypot(d['dock_long'], d['dock_lat']):.2f} m / {abs(d['dock_head_deg']):.1f} deg" for d in docks)
    table_card(f'{tmp}/r.png', 'Planning: results', ['path', 'cusps', 'nodes', 'time'], rows,
               ['Reeds-Shepp analytic expansion (gear changes only where the goal needs a turnaround) + RS shortcutting.',
                'Heuristic: max(obstacle-aware Dijkstra, Reeds-Shepp distance); soft clearance cost keeps the AGV centred.',
                'Live occupancy from perception: blocked ahead -> stop line, hold 2.5 s, replan if it stays.',
                f'Tracked by the MPC: arrival error {dock_txt}; lateral RMS {100 * ctl[-1]["lat_rms"]:.0f} cm.'],
               'Plan times measured live during the recording (single core, with RViz running); 0.5 m site costmap, 72 headings',
               col_x=(1010, 1220, 1390, 1600))
    caption_png(f'{tmp}/c.png', 3, 'PLANNING  ·  Hybrid A* + Reeds-Shepp, stop-and-replan around an unmapped obstacle')
    still_clip(f'{tmp}/t.png', f'{tmp}/a.mp4', 3.5)
    main_clip(os.path.join(RAW, 'planning_raw.mp4'), f'{tmp}/b.mp4', 1.2, 96.0, f'{tmp}/c.png', speed=1.5)
    still_clip(f'{tmp}/r.png', f'{tmp}/c.mp4', 8.0)
    out = os.path.join(MEDIA, '03_planning.mp4')
    concat([f'{tmp}/a.mp4', f'{tmp}/b.mp4', f'{tmp}/c.mp4'], out)
    os.replace(f'{tmp}/r.png', os.path.join(MEDIA, '03_planning_results.png'))
    print('wrote', out)


def multi_speed_clip(src, out, segments, caption=None):
    """segments: (start, end, speed) in source seconds -> one clip with fades at both ends."""
    parts, filt = [], []
    for k, (a, b, sp) in enumerate(segments):
        filt.append(f'[0:v]trim=start={a}:end={b},setpts=(PTS-STARTPTS)/{sp}[s{k}]')
        parts.append(f'[s{k}]')
    total = sum((b - a) / sp for a, b, sp in segments)
    filt.append(''.join(parts) + f'concat=n={len(segments)}:v=1:a=0[m]')
    inputs = ['-i', src]
    last = 'm'
    if caption:
        inputs += ['-i', caption]
        filt.append('[m][1:v]overlay=0:0[n]')
        last = 'n'
    filt.append(f'[{last}]fade=t=in:st=0:d=0.5,fade=t=out:st={total - 0.6}:d=0.6,format=yuv420p[v]')
    run(['ffmpeg', '-y', *inputs, '-filter_complex', ';'.join(filt), '-map', '[v]', '-r', str(FPS), '-an', '-c:v', 'libx264',
         '-preset', 'medium', '-crf', '21', out])


def compare_card(path, title, left, right, left_label, right_label, footer):
    im, d = _canvas()
    d.text((150, 80), title, font=font(56, 'B'), fill=FG)
    for k, (src, lab) in enumerate(((left, left_label), (right, right_label))):
        img = Image.open(src).convert('RGB').resize((780, 439))
        x = 150 + k * 840
        im.paste(img, (x, 230))
        d.text((x, 690), lab, font=font(30, 'M'), fill=FG if k else DIM)
    d.text((150, H - 100), footer, font=font(24, 'R'), fill=DIM)
    im.save(path)


def build_calibration():
    m = last_metrics(os.path.join(LOGS, 'calibration_rec.jsonl'), '/calibration/metrics')
    tmp = tempfile.mkdtemp()
    title_card(f'{tmp}/t.png', 5, 'Calibration', 'Four corner LiDARs with wrong extrinsics, no targets',
               ['Big vehicles need corner LiDARs for the blind spots a roof LiDAR cannot see',
                'TF as installed is off by up to 3.6 deg / 31 cm.  Stage 1 parked: ground plane (roll, pitch, height)',
                'Stage 2 driving: keyframes registered jointly to a roof-LiDAR map (yaw, x, y).   Drive shown at 2.5x'])
    for t, name in ((76.0, 'before'), (90.0, 'after')):
        run(['ffmpeg', '-y', '-ss', str(t), '-i', os.path.join(RAW, 'calibration_raw.mp4'), '-frames:v', '1', f'{tmp}/{name}.png'])
    compare_card(f'{tmp}/k.png', 'Same keyframes, before and after', f'{tmp}/before.png', f'{tmp}/after.png',
                 'before: extrinsics as installed (ghost walls)', 'after calibration: points on the surfaces',
                 'Blue: front corner LiDARs  ·  orange: rear corner LiDARs  ·  every keyframe re-projected with the current estimate')
    lid = m['lidars']
    rows = [(n.replace('_', ' '), [f"{e['rot_before_deg']:4.2f} deg", f"{e['rot_deg']:4.2f} deg", f"{e['trans_before_cm']:4.1f} cm",
                                   f"{e['trans_cm']:4.1f} cm"], (236, 240, 245)) for n, e in sorted(lid.items())]
    table_card(f'{tmp}/r.png', 'Calibration: results', ['rot. before', 'after', 'trans. before', 'after'], rows,
               ['Ground plane (RANSAC + PCA) fixes roll, pitch and height exactly: it needs nothing but flat ground.',
                'Joint point-to-plane Gauss-Newton over all keyframes of a sensor against the roof-LiDAR map: yaw, x, y.',
                'Observability matters: a straight drive past parallel walls leaves x unconstrained (4-8 cm error);',
                'adding a 90 deg turn past perpendicular stack faces brings it to 1-3 cm. Errors vs the simulator truth.'],
               f"{m['keyframes']} keyframes, {m['iteration']} iterations  ·  outputs calibrated extrinsics as YAML for the TF tree",
               col_x=(760, 1030, 1270, 1560))
    caption_png(f'{tmp}/c.png', 5, 'CALIBRATION  ·  target-less multi-LiDAR extrinsics: ground plane + mapping-based registration')
    still_clip(f'{tmp}/t.png', f'{tmp}/a.mp4', 3.5)
    multi_speed_clip(os.path.join(RAW, 'calibration_raw.mp4'), f'{tmp}/b.mp4', [(1.0, 9.0, 1.0), (9.0, 76.0, 2.5), (76.0, 91.5, 1.0)],
                     f'{tmp}/c.png')
    still_clip(f'{tmp}/k.png', f'{tmp}/c.mp4', 6.0)
    still_clip(f'{tmp}/r.png', f'{tmp}/d.mp4', 8.0)
    out = os.path.join(MEDIA, '05_calibration.mp4')
    concat([f'{tmp}/a.mp4', f'{tmp}/b.mp4', f'{tmp}/c.mp4', f'{tmp}/d.mp4'], out)
    os.replace(f'{tmp}/r.png', os.path.join(MEDIA, '05_calibration_results.png'))
    os.replace(f'{tmp}/k.png', os.path.join(MEDIA, '05_calibration_before_after.png'))
    print('wrote', out)


def build_mission():
    log = os.path.join(LOGS, 'mission_rec.jsonl')
    rows = [json.loads(line) for line in open(log)]
    last = lambda topic: [r['data'] for r in rows if r['topic'] == topic][-1]  # noqa: E731
    pe, lo, co, pl, sd = (last(t) for t in ('/perception/metrics', '/localization/metrics', '/control/metrics',
                                             '/planning/metrics', '/sim/debug'))
    ms = [(r['stamp'] - 100.0, r['data']) for r in rows if r['topic'] == '/mission/status']
    t_done = next(t for t, x in ms if x['state'] == 'dwell' and x['goal'] == 'dump')
    yaw = math.degrees(sd['yaw'])
    head = yaw - 180 if yaw > 0 else yaw + 180
    scales = [r['data']['odom_scale'] for r in rows if r['topic'] == '/localization/metrics']
    tmp = tempfile.mkdtemp()
    title_card(f'{tmp}/t.png', 6, 'Full stack', 'One quarry haul cycle, closed loop',
               ['Perception, EKF + NDT localization, Hybrid A* planning and MPC on one 72 t truck',
                'Back in beside the excavator, haul 330 m up a 9 % ramp through dust, mud and traffic, reverse into the dump bay',
                'Haul shown at 2.5x'])
    W_ = (236, 240, 245)
    table_card(f'{tmp}/r.png', 'Full stack: one haul cycle', ['measured'], [
        ('Mission time (load + haul + dump)', [f'{t_done:4.0f} s'], W_),
        ('Perception: dust echoes removed', [f"{100 * pe['avg_dust_recall_dusty']:4.1f} %"], W_),
        ('Perception: latency per frame', [f"{pe['avg_ms']:4.1f} ms"], W_),
        ('Localization: position error RMS / max', [f"{100 * lo['fused']['rms']:3.0f} / {100 * lo['fused']['max']:3.0f} cm"], W_),
        ('Control: lateral error RMS', [f"{100 * co['lat_rms']:3.0f} cm"], W_),
        ('Planning: replans around traffic / obstacles', [f"{pl['replans']}"], W_),
        ('Docking at the berm: position / heading', [f"{math.hypot(167.5 - sd['x'], sd['y'] - 3.0):4.2f} m / {abs(head):3.1f} deg"], W_)],
        [f'Wheel slip in the mud is tracked by the EKF odometry-scale state ({min(scales):.2f}-{max(scales):.2f} during this run).',
         'Live occupancy is cluster-validated and near-field only; the planner stops, waits, then replans.',
         'Everything runs on one machine in real time, in sim time, with RViz software-rendering this view.'],
        'Numbers from this recorded run, scored against simulator ground truth', col_x=(1340,))
    caption_png(f'{tmp}/c.png', 6, 'FULL STACK  ·  perception + localization + planning + control, one haul cycle')
    still_clip(f'{tmp}/t.png', f'{tmp}/a.mp4', 3.5)
    multi_speed_clip(os.path.join(RAW, 'mission_raw.mp4'), f'{tmp}/b.mp4', [(1.0, 33.0, 1.0), (33.0, 132.0, 2.5), (132.0, 160.0, 1.0)],
                     f'{tmp}/c.png')
    still_clip(f'{tmp}/r.png', f'{tmp}/c.mp4', 8.0)
    out = os.path.join(MEDIA, '06_full_mission.mp4')
    concat([f'{tmp}/a.mp4', f'{tmp}/b.mp4', f'{tmp}/c.mp4'], out)
    os.replace(f'{tmp}/r.png', os.path.join(MEDIA, '06_full_mission_results.png'))
    print('wrote', out)


def excerpt(src, out, start, dur):
    run(['ffmpeg', '-y', '-ss', str(start), '-i', src, '-t', str(dur), '-vf',
         f'fade=t=in:st=0:d=0.4,fade=t=out:st={dur - 0.5}:d=0.5,format=yuv420p', '-r', str(FPS), '-an',
         '-c:v', 'libx264', '-preset', 'medium', '-crf', '22', out])


def build_reel():
    tmp = tempfile.mkdtemp()
    im, d = _canvas()
    d.text((150, 330), 'Industrial & off-road', font=font(84, 'B'), fill=FG)
    d.text((150, 430), 'autonomous driving stack', font=font(84, 'B'), fill=FG)
    d.text((154, 560), 'Perception  ·  Localization  ·  Calibration  ·  Planning  ·  Control', font=font(38, 'L'), fill=ACC)
    d.text((154, 630), 'ROS 2 Humble  ·  C++17  ·  3D sensor-level simulation of a quarry and a container terminal', font=font(28, 'R'), fill=DIM)
    d.text((154, 680), 'Dust, rough terrain, GNSS multipath, 0.5 s actuator dead time, no lanes', font=font(28, 'R'), fill=DIM)
    im.save(f'{tmp}/intro.png')
    rows = [('Perception', 'dust echoes removed 98 %; 29x fewer false obstacles than a RANSAC baseline'),
            ('Localization', '8 cm RMS under port GNSS multipath and crane outage (GNSS-based: 2.4 m)'),
            ('Planning', 'Hybrid A* + Reeds-Shepp: 160-230 m in 0.1-0.6 s; stop and replan on unmapped obstacles'),
            ('Control', 'MPC 0.13 m lateral RMS at 0.5 s dead time (pure pursuit: 0.90 m); docks to 0.41 m / 1.3 deg'),
            ('Calibration', '4 corner LiDARs, target-less: 3.6 deg / 31 cm  ->  0.08 deg / 2.6 cm'),
            ('Full stack', 'load, haul 330 m, dump: one closed-loop cycle in 157 s, localization 5 cm RMS')]
    im, d = _canvas()
    d.text((150, 120), 'Results at a glance', font=font(64, 'B'), fill=FG)
    y = 270
    for k, v in rows:
        d.line([(150, y - 14), (W - 150, y - 14)], fill=(40, 45, 55), width=2)
        d.text((150, y), k, font=font(34, 'M'), fill=ACC)
        d.text((520, y + 4), v, font=font(30, 'R'), fill=FG)
        y += 92
    d.text((150, H - 110), 'Every number scored against simulator ground truth  ·  code, unit tests and full-length videos in the repository',
           font=font(24, 'R'), fill=DIM)
    im.save(f'{tmp}/outro.png')
    still_clip(f'{tmp}/intro.png', f'{tmp}/00.mp4', 4.0)
    cuts = [('01_perception.mp4', 16.0, 15.0), ('02_localization.mp4', 28.0, 14.0), ('03_planning.mp4', 17.0, 14.0),
            ('04_control.mp4', 40.0, 16.0), ('05_calibration.mp4', 36.0, 14.0), ('06_full_mission.mp4', 42.0, 18.0)]
    clips = [f'{tmp}/00.mp4']
    for k, (f, a, dur) in enumerate(cuts):
        excerpt(os.path.join(MEDIA, f), f'{tmp}/{k + 1:02d}.mp4', a, dur)
        clips.append(f'{tmp}/{k + 1:02d}.mp4')
    still_clip(f'{tmp}/outro.png', f'{tmp}/99.mp4', 9.0)
    clips.append(f'{tmp}/99.mp4')
    out = os.path.join(MEDIA, 'showcase_reel.mp4')
    concat(clips, out)
    os.replace(f'{tmp}/outro.png', os.path.join(MEDIA, 'results_at_a_glance.png'))
    print('wrote', out)


def build_compress(clips=None):
    """Compress explicit clips, or media/*.mp4 when called without arguments."""
    sources = clips if clips is not None else [os.path.join(MEDIA, f) for f in sorted(os.listdir(MEDIA))
                                               if f.endswith('.mp4')]
    for source in sources:
        src = os.path.abspath(source)
        f = os.path.basename(src)
        tmp = src + '.tmp.mp4'
        run(['ffmpeg', '-y', '-i', src, '-c:v', 'libx264', '-preset', 'slow', '-crf', '24', '-maxrate', '7M', '-bufsize', '14M',
             '-pix_fmt', 'yuv420p', '-movflags', '+faststart', '-an', tmp])
        before, after = os.path.getsize(src), os.path.getsize(tmp)
        if after < before:
            os.replace(tmp, src)
        else:
            os.unlink(tmp)
        print(f'{f}: {before / 1e6:5.1f} MB -> {min(before, after) / 1e6:5.1f} MB')


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help'):
        print('''Usage: python3 tools/video.py COMMAND [ARGS]

V3 (Gazebo Harmonic + RViz):
  v3-retime NAME [--raw-dir DIR] [--capture-offset SECONDS]
  v3-compose NAME {pip,pip_left,side,gz,rviz} OUT.mp4 [--raw-dir DIR]
  v3-card RESULTS.json OUT.png
  v3-demo NAME LAYOUT RESULTS.json OUT.mp4 [--raw-dir DIR] [--capture-offset SECONDS]
  v3-reel OUT.mp4 CLIP1.mp4 [CLIP2.mp4 ...]  (0.5 s fades through black)
  compress [CLIP.mp4 ...]                   (default: media/*.mp4)

V2: perception, localization, planning, control, calibration, mission, reel
Self-test: python3 tools/video_v3_selftest.py (temporary inputs in /tmp)''')
    elif sys.argv[1].startswith('v3-'):
        from video_v3 import main
        main(sys.argv[1:])
    elif sys.argv[1] == 'compress':
        build_compress(sys.argv[2:] or None)
    else:
        command = globals().get('build_' + sys.argv[1])
        if not callable(command):
            sys.exit(f'Unknown command: {sys.argv[1]} (see --help)')
        command()
