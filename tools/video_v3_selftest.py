#!/usr/bin/env python3
"""Exercise the v3 CLI with synthetic 1080p recordings; keep all artifacts in /tmp.

    python3 tools/video_v3_selftest.py
"""
import json
from fractions import Fraction
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

from video_v3 import source_frame_indices


VIDEO = Path(__file__).with_name('video.py')
FONT = '/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf'


def run(*args):
    result = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(f"Command failed: {args!r}\n{result.stdout}\n{result.stderr}")
    return result.stdout


def cli(*args):
    print('  ' + ' '.join(str(a) for a in args), flush=True)
    return run(sys.executable, VIDEO, *args)


def check_video(path, duration):
    data = json.loads(run('ffprobe', '-v', 'error', '-select_streams', 'v:0',
                          '-show_streams', '-show_format', '-of', 'json', path))
    stream = data['streams'][0]
    actual = float(stream.get('duration', data['format']['duration']))
    assert abs(actual - duration) <= 2 / 30, (path, actual, duration)
    assert (stream['width'], stream['height']) == (1920, 1080), stream
    assert stream['codec_name'] == 'h264', stream
    assert stream['pix_fmt'] == 'yuv420p', stream
    assert Fraction(stream['avg_frame_rate']) == 30, stream
    print(f'    {Path(path).name}: {actual:.3f} s, 1920x1080, H.264, 30 fps', flush=True)


def synthetic(path, duration, label):
    run('ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
        '-f', 'lavfi', '-i', f'testsrc2=size=1920x1080:rate=30:duration={duration}',
        '-vf', f"drawtext=fontfile={FONT}:text='{label} %{{pts\\:hms}}':"
               'x=40:y=40:fontsize=60:fontcolor=white:box=1:boxcolor=black@0.8,'
               'drawbox=x=0:y=0:w=60:h=60:color=white:t=fill',
        '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '18',
        '-threads', '2', '-pix_fmt', 'yuv420p', path)


def frame(path, number):
    result = subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-i', str(path),
        '-vf', f'select=eq(n\\,{number}),scale=320:180', '-frames:v', '1',
        '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-threads', '1', 'pipe:1',
    ], check=True, capture_output=True)
    return np.frombuffer(result.stdout, dtype=np.uint8).astype(np.float32)


def check_mapping():
    # A second pause must jump from wall 4 to wall 6 at sim .6, without
    # stretching the preceding advancing segment over that paused interval.
    indices = source_frame_indices(np.array([0., 3., 4., 6., 8.]),
                                   np.array([0., 0., .6, .6, 2.4]), 8.)
    for output, source in ((0, 90), (9, 105), (18, 180), (27, 190)):
        assert abs(int(indices[output]) - source) <= 1, (output, indices[output], source)
    offset = source_frame_indices(np.array([0., 3., 7.]),
                                 np.array([0., 0., 2.4]), 7., capture_offset=.2)
    assert abs(int(offset[0]) - 84) <= 1, offset[0]
    fast = source_frame_indices(np.array([0., 1., 2.]), np.array([0., 2., 4.]), 2.)
    assert np.any(np.diff(fast) == 0), 'RTF > 1 must duplicate source frames'
    assert np.all(np.diff(indices) >= 0), 'Source decoding must remain sequential'


def check_pip_border(path, x):
    # The white fixture corner distinguishes the inset from its blue border.
    # Inspect luma so final yuv420p chroma subsampling cannot blur the boundary.
    result = subprocess.run([
        'ffmpeg', '-hide_banner', '-loglevel', 'error', '-i', str(path),
        '-vf', f'select=eq(n\\,0),extractplanes=y,crop=16:16:{x}:24',
        '-frames:v', '1', '-f', 'rawvideo', '-pix_fmt', 'gray',
        '-threads', '1', 'pipe:1',
    ], check=True, capture_output=True)
    corner = np.frombuffer(result.stdout, dtype=np.uint8).reshape(16, 16)
    assert np.all(corner[:3, :] < 210), ('PiP top border must be 3 px', corner.tolist())
    assert np.all(corner[:, :3] < 210), ('PiP left border must be 3 px', corner.tolist())
    assert np.all(corner[3:13, 3:13] > 220), ('PiP content must start at (3, 3)', corner.tolist())


def main():
    print('Checking clock interpolation, pauses, offset, and duplicated frames...', flush=True)
    check_mapping()
    with tempfile.TemporaryDirectory(prefix='video-v3-selftest-', dir='/tmp') as tmp:
        root = Path(tmp)
        raw = root / 'raw'
        raw.mkdir()
        print('Generating timestamped 1080p recordings in /tmp...', flush=True)
        synthetic(raw / 'synthetic.mp4', 7, 'wall')
        synthetic(raw / 'synthetic_gz.mp4', 2, 'sim')
        with (raw / 'synthetic_clock.txt').open('w') as clock:
            for wall in np.arange(141) / 20:
                clock.write(f'{1_700_000_000 + wall:.6f} {max(0., wall - 3) * .6:.6f}\n')
        results = root / 'results.json'
        results.write_text(json.dumps({
            'title': 'Synthetic v3 demo',
            'subtitle': 'Gazebo Harmonic + RViz; 0.6 real-time factor',
            'rows': [['Paused lead-in', '3 s'], ['Input / output', '7 s', '2.4 s'],
                     ['Real-time factor', 0.6]],
            'note': 'Synthetic integration check. All artifacts are temporary.',
        }), encoding='utf-8')

        cli('v3-retime', 'synthetic', '--raw-dir', raw)
        retimed = raw / 'synthetic_rviz_sim.mp4'
        check_video(retimed, 2.4)
        for output_frame in (0, 18, 54):
            actual = frame(retimed, output_frame)
            expected = 90 + round(output_frame / .6)
            error = min(float(np.mean(np.abs(actual - frame(raw / 'synthetic.mp4', source))))
                        for source in range(expected - 1, expected + 2))
            assert error < 5, (output_frame, 'retimed frame differs from clock mapping', error)

        clips = []
        for layout in ('pip', 'pip_left', 'side', 'gz', 'rviz'):
            out = root / f'{layout} result.mp4'
            cli('v3-compose', 'synthetic', layout, out, '--raw-dir', raw)
            check_video(out, 2)
            if layout in ('pip', 'pip_left'):
                check_pip_border(out, 1250 if layout == 'pip' else 24)
            clips.append(out)

        png = root / 'card.png'
        cli('v3-card', results, png)
        with Image.open(png) as card:
            assert card.size == (1920, 1080), card.size
            assert np.ptp(np.asarray(card)) > 100, 'Card should contain visible text'

        demo = root / 'demo.mp4'
        cli('v3-demo', 'synthetic', 'pip', results, demo, '--raw-dir', raw)
        check_video(demo, 10)
        reel = root / 'reel.mp4'
        cli('v3-reel', reel, clips[0], clips[2])
        check_video(reel, 4)
        assert float(frame(reel, 0).mean()) < 5, 'Reel should fade in from black'
        assert float(frame(reel, 59).mean()) < 20, 'First clip should fade to black'
        assert float(frame(reel, 60).mean()) < 5, 'Second clip should fade in from black'

        cli('compress', reel)
        check_video(reel, 4)
    print('v3 self-test passed; synthetic files deleted.', flush=True)


if __name__ == '__main__':
    main()
