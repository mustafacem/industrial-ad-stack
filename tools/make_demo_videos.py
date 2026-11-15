#!/usr/bin/env python3
"""Assemble the v3 demo videos from output/raw/ recordings (scripts/make_videos.sh) with tools/video_v3.py:
title card -> composed run (RViz + Gazebo chase camera on sim time) -> results card; then the reel and GIFs.

  python3 tools/make_demo_videos.py [perception localization planning control calibration mission reel gifs]
"""
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import video_v3 as v3  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RAW, RES, MEDIA = ROOT / 'output' / 'raw', ROOT / 'output' / 'results', ROOT / 'media'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
if not os.path.exists(FONT):
    FONT = os.path.join(v3.v2.FONT_DIR, 'Ubuntu-B.ttf')


def ff(*args):
    subprocess.run(['ffmpeg', '-v', 'error', '-y', *map(str, args)], check=True)


def speed(src, dst, factor, label=True):
    vf = f'setpts=PTS/{factor},fps=30'
    if label:
        vf += (f",drawtext=fontfile='{FONT}':text='{factor:g}x':x=w-110:y=h-70:fontsize=40:"
               "fontcolor=0xecf0f5:box=1:boxcolor=0x0e1015@0.7:boxborderw=10")
    ff('-i', src, '-vf', vf, '-an', '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p', dst)


def demo(name, raw, layout='pip', factor=1.0):
    results = RES / f'{name}.json'
    data = v3._load_results(results)
    with tempfile.TemporaryDirectory(prefix='demo-') as tmp:
        tmp = Path(tmp)
        v3.retime_rviz(raw)
        v3.compose(raw, layout, tmp / 'body.mp4')
        body = tmp / 'body.mp4'
        if factor != 1.0:
            speed(body, tmp / 'body_fast.mp4', factor)
            body = tmp / 'body_fast.mp4'
        v3._title_card(data, tmp / 'title.png')
        v3.render_results_card(results, tmp / 'results.png')
        v3.card_clip(tmp / 'title.png', tmp / 'title.mp4', 3)
        v3.card_clip(tmp / 'results.png', tmp / 'results.mp4', 6)
        v3._concat([tmp / 'title.mp4', body, tmp / 'results.mp4'], MEDIA / f'{name}.mp4')
        ff('-i', tmp / 'results.png', '-frames:v', '1', MEDIA / f'{name}_results.png')
    print('wrote', MEDIA / f'{name}.mp4')


def control():
    """2x2 grid on the common sim-time axis: [MPC RViz | MPC Gazebo] / [pure-pursuit RViz | pure-pursuit Gazebo]."""
    name = '04_control'
    data = v3._load_results(RES / f'{name}.json')
    with tempfile.TemporaryDirectory(prefix='ctl-') as tmp:
        tmp = Path(tmp)
        for run in ('04_control_mpc', '04_control_pp'):
            v3.retime_rviz(run)
        lab = lambda t: (f"drawtext=fontfile='{FONT}':text='{t}':x=20:y=h-56:fontsize=30:fontcolor=0xecf0f5:"
                         "box=1:boxcolor=0x0e1015@0.75:boxborderw=8")
        fc = ';'.join([
            f'[0:v]setpts=PTS-STARTPTS,fps=30,scale=960:540,{lab("MPC + dead-time compensation")}[a]',
            '[1:v]setpts=PTS-STARTPTS,fps=30,scale=960:540[b]',
            f'[2:v]setpts=PTS-STARTPTS,fps=30,scale=960:540,{lab("pure pursuit, no compensation")}[c]',
            '[3:v]setpts=PTS-STARTPTS,fps=30,scale=960:540[d]',
            '[a][b]hstack=inputs=2:shortest=1[top]', '[c][d]hstack=inputs=2:shortest=1[bot]',
            '[top][bot]vstack=inputs=2:shortest=1[v]'])
        ff('-i', RAW / '04_control_mpc_rviz_sim.mp4', '-i', RAW / '04_control_mpc_gz.mp4',
           '-i', RAW / '04_control_pp_rviz_sim.mp4', '-i', RAW / '04_control_pp_gz.mp4',
           '-filter_complex', fc, '-map', '[v]', '-an', '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p',
           tmp / 'body.mp4')
        v3._title_card(data, tmp / 'title.png')
        v3.render_results_card(RES / f'{name}.json', tmp / 'results.png')
        v3.card_clip(tmp / 'title.png', tmp / 'title.mp4', 3)
        v3.card_clip(tmp / 'results.png', tmp / 'results.mp4', 6)
        v3._concat([tmp / 'title.mp4', tmp / 'body.mp4', tmp / 'results.mp4'], MEDIA / f'{name}.mp4')
        ff('-i', tmp / 'results.png', '-frames:v', '1', MEDIA / f'{name}_results.png')
    print('wrote', MEDIA / f'{name}.mp4')


def reel():
    """~2 min: from each demo its title card and a cut of the running system (no results cards)."""
    cuts = [('01_perception', 3, 22), ('02_localization', 3, 20), ('03_planning', 3, 20),
            ('04_control', 3, 18), ('05_calibration', 3, 18), ('06_full_mission', 3, 24)]
    with tempfile.TemporaryDirectory(prefix='reel-') as tmp:
        parts = []
        for name, title_s, body_s in cuts:
            src = MEDIA / f'{name}.mp4'
            if not src.exists():
                continue
            p = Path(tmp) / f'{name}.mp4'
            # title card + a slice of the body that starts 25 % into the run
            dur = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', str(src)],
                                       capture_output=True, text=True).stdout)
            start = title_s + 0.25 * max(0.0, dur - title_s - 6 - body_s)
            ff('-i', src, '-filter_complex',
               f'[0:v]trim=0:{title_s},setpts=PTS-STARTPTS[t];[0:v]trim={start}:{start + body_s},setpts=PTS-STARTPTS[b];'
               '[t][b]concat=n=2:v=1[v]', '-map', '[v]', '-an', '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p', p)
            parts.append(p)
        v3.reel(MEDIA / 'showcase_reel.mp4', parts)
    print('wrote', MEDIA / 'showcase_reel.mp4')


def gifs():
    (MEDIA / 'gif').mkdir(exist_ok=True)
    for src in sorted(MEDIA.glob('0*.mp4')):
        dur = float(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'format=duration', '-of', 'csv=p=0', str(src)],
                                   capture_output=True, text=True).stdout)
        start = 3 + 0.3 * max(0.0, dur - 9 - 6)
        out = MEDIA / 'gif' / (src.stem + '.gif')
        ff('-ss', f'{start:.1f}', '-t', '6', '-i', src, '-vf',
           'fps=8,scale=480:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=bayer:bayer_scale=3',
           out)
        print('wrote', out)


JOBS = {
    'perception': lambda: demo('01_perception', '01_perception', layout='pip_br'),
    'localization': lambda: demo('02_localization', '02_localization', factor=2.0),
    'planning': lambda: demo('03_planning', '03_planning', factor=2.0),
    'control': control,
    'calibration': lambda: demo('05_calibration', '05_calibration', factor=1.5),
    'mission': lambda: demo('06_full_mission', '06_mission', factor=2.0),
    'reel': reel,
    'gifs': gifs,
}

if __name__ == '__main__':
    MEDIA.mkdir(exist_ok=True)
    for job in sys.argv[1:] or list(JOBS):
        JOBS[job]()
