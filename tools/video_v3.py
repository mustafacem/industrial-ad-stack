"""Gazebo Harmonic / RViz finishing, using the v2 card palette and fonts.

The default capture start is the first clock sample. Use --capture-offset when
the RViz recorder's actual start relative to that sample is known (seconds).
Video is silent; reels use 0.5 s fades through black without overlapping clips.
"""
import argparse
from contextlib import contextmanager
from fractions import Fraction
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np

try:  # Both `python tools/video.py` and `import tools.video_v3`.
    from . import video as v2
except ImportError:
    import video as v2

W, H, FPS = v2.W, v2.H, v2.FPS
LAYOUTS = ('pip', 'pip_left', 'side', 'gz', 'rviz', 'pip_br')
FFMPEG = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
          '-filter_complex_threads', '1', '-filter_threads', '1']


def _input(path):
    return ['-threads', '2', '-i', str(path)]


def _encoding():
    return ['-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
            '-threads', '4', '-pix_fmt', 'yuv420p', '-r', str(FPS),
            '-video_track_timescale', '15360', '-movflags', '+faststart']


def _probe(path):
    data = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
        'stream=width,height,avg_frame_rate,nb_frames,duration:format=duration',
        '-of', 'json', str(path)]))
    stream = data['streams'][0]
    rate = float(Fraction(stream['avg_frame_rate']))
    count = stream.get('nb_frames', 'N/A')
    duration = (int(count) / rate if count != 'N/A' else
                float(stream.get('duration', data['format'].get('duration', 0))))
    if not math.isfinite(duration) or duration <= 0 or rate <= 0:
        raise ValueError(f'{path}: no usable video duration/frame rate')
    return stream, duration, rate


def _paths(name, raw_dir=None):
    if not name or Path(name).name != name or name in ('.', '..'):
        raise ValueError('name must be a recording basename, without directories')
    raw = Path(raw_dir) if raw_dir is not None else Path(v2.RAW)
    return (raw / f'{name}.mp4', raw / f'{name}_gz.mp4',
            raw / f'{name}_clock.txt', raw / f'{name}_rviz_sim.mp4')


@contextmanager
def _destination(out, sources=()):
    """Keep an earlier output intact if ffmpeg fails; never overwrite inputs."""
    out = Path(out).resolve()
    if out in (Path(p).resolve() for p in sources):
        raise ValueError(f'Output would overwrite an input: {out}')
    out.parent.mkdir(parents=True, exist_ok=True)
    fd, temp = tempfile.mkstemp(prefix=f'.{out.stem}-', suffix=out.suffix, dir=out.parent)
    os.close(fd)
    try:
        yield temp
        os.replace(temp, out)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def _write_video(args, out, sources=()):
    with _destination(out, sources) as temp:
        subprocess.run(FFMPEG + args + [temp], check=True, stdout=subprocess.DEVNULL)
    return str(out)


def source_frame_indices(wall, sim, source_duration, source_fps=FPS, capture_offset=0.0):
    """Inverse piecewise-linear clock mapping, with one index per sim-time frame.

    Wall samples are relative to the first log entry. Repeated sim timestamps
    represent pauses: at that exact time choose the *last* paused wall sample.
    Keep both ends of every plateau to avoid stretching adjacent motion.
    """
    wall, sim = np.asarray(wall, dtype=float), np.asarray(sim, dtype=float)
    if (wall.ndim != 1 or sim.shape != wall.shape or len(wall) < 2 or
            not np.all(np.isfinite(wall)) or not np.all(np.isfinite(sim)) or
            np.any(np.diff(wall) <= 0) or np.any(np.diff(sim) < 0)):
        raise ValueError('Clock needs finite, increasing wall times and nondecreasing sim times (no resets)')
    if (not math.isfinite(capture_offset) or not math.isfinite(source_duration) or
            source_duration <= 0 or not math.isfinite(source_fps) or source_fps <= 0):
        raise ValueError('Invalid capture offset, video duration or frame rate')
    start_wall = max(wall[0], capture_offset)
    end_wall = min(wall[-1], capture_offset + source_duration)
    if end_wall <= start_wall:
        raise ValueError('Clock log and RViz capture do not overlap')
    start_sim, end_sim = np.interp([start_wall, end_wall], wall, sim)
    count = math.ceil((end_sim - start_sim) * FPS - 1e-7)
    if count <= 0:
        raise ValueError('No advancing simulation time within the RViz capture')
    targets = start_sim + np.arange(count) / FPS
    upper = np.searchsorted(sim, targets, side='right')
    lower = upper - 1
    times = wall[lower] + ((targets - sim[lower]) / (sim[upper] - sim[lower]) *
                           (wall[upper] - wall[lower]))
    indices = np.floor((times - capture_offset) * source_fps + .5).astype(np.int64)
    return np.clip(indices, 0, max(0, math.ceil(source_duration * source_fps - 1e-7) - 1))


def _stop(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    process.wait()


def retime_rviz(name, raw_dir=None, capture_offset=0.0):
    """Stream YUV frames through ffmpeg without storing decoded clips in RAM."""
    source, _, clock, out = _paths(name, raw_dir)
    stream, duration, rate = _probe(source)
    if (stream['width'], stream['height']) != (W, H):
        raise ValueError(f'RViz capture must be {W}x{H}')
    samples = np.loadtxt(clock, ndmin=2)
    if samples.shape[1] != 2:
        raise ValueError(f'{clock}: expected two columns: wall_time_s sim_time_s')
    wall = samples[:, 0] - samples[0, 0]
    indices = source_frame_indices(wall, samples[:, 1], duration, rate, capture_offset)
    print(f'Retiming {name}: {duration:.3f} s wall -> {len(indices) / FPS:.3f} s sim '
          f'(capture offset {capture_offset:+.3f} s)', flush=True)
    frame_bytes = W * H * 3 // 2
    with _destination(out, [source, clock]) as temp:
        decoder = subprocess.Popen(FFMPEG + _input(source) + [
            '-map', '0:v:0', '-an', '-vsync', '0', '-f', 'rawvideo',
            '-pix_fmt', 'yuv420p', '-threads', '1', 'pipe:1'], stdout=subprocess.PIPE)
        encoder = None
        try:
            encoder = subprocess.Popen(FFMPEG + [
                '-f', 'rawvideo', '-pixel_format', 'yuv420p', '-video_size', f'{W}x{H}',
                '-framerate', str(FPS), '-i', 'pipe:0'] + _encoding() + [temp],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
            current, frame = -1, None
            for wanted in indices:
                while current < wanted:
                    frame = decoder.stdout.read(frame_bytes)
                    if len(frame) != frame_bytes:
                        raise RuntimeError(f'{source}: decoder ended at frame {current + 1}, needed {wanted}')
                    current += 1
                encoder.stdin.write(frame)
            encoder.stdin.close()
            if encoder.wait():
                raise RuntimeError('ffmpeg failed to encode the retimed RViz capture')
        finally:
            decoder.stdout.close()
            _stop(decoder)  # Its unused tail must not block on a full pipe.
            if encoder is not None:
                try:
                    if not encoder.stdin.closed:
                        encoder.stdin.close()
                except BrokenPipeError:
                    pass
                finally:
                    _stop(encoder)
    return str(out)


def compose(name, layout, out, raw_dir=None):
    """Compose on sim time; even single-view layouts use the common duration."""
    if layout not in LAYOUTS:
        raise ValueError(f'Unknown layout: {layout}')
    source, gz, clock, rviz = _paths(name, raw_dir)
    _, rviz_duration, _ = _probe(rviz)
    _, gz_duration, _ = _probe(gz)
    frames = math.floor(min(rviz_duration, gz_duration) * FPS + 1e-7)
    if frames < 1:
        raise ValueError('Inputs have less than one common frame')
    normal = f'setpts=PTS-STARTPTS,fps={FPS},scale={W}:{H},setsar=1'
    if layout in ('gz', 'rviz'):
        args = _input(gz if layout == 'gz' else rviz) + ['-vf', normal]
    else:
        filters = [f'[0:v]{normal}[r]', f'[1:v]{normal}[g]']
        if layout == 'side':
            filters += ['[g]scale=960:540,pad=960:1080:0:270:color=0x0e1015[left]',
                        '[r]crop=960:1080:480:0[right]', '[left][right]hstack=inputs=2:shortest=1[v]']
        else:
            x = 24 if layout == 'pip_left' else W - 646 - 24
            y = H - 366 - 24 if layout == 'pip_br' else 24  # bottom-right: keeps split-view HUDs visible
            font_path = os.path.join(v2.FONT_DIR, 'Ubuntu-R.ttf')
            # 4:4:4 preserves odd padding offsets for the exact 3 px border.
            filters += ['[g]scale=640:360,format=yuv444p,pad=646:366:3:3:color=0x6ec1ff[inset]',
                        f'[r][inset]overlay=x={x}:y={y}:shortest=1:eof_action=endall:format=yuv444,'
                        f'drawbox=x={x + 3}:y={y + 321}:w=640:h=42:color=0x0e1015@0.85:t=fill,'
                        f"drawtext=fontfile='{font_path}':text='Gazebo Harmonic - chase camera':"
                        f'x={x + 15}:y={y + 328}:fontsize=25:fontcolor=0xecf0f5[v]']
        args = _input(rviz) + _input(gz) + ['-filter_complex', ';'.join(filters), '-map', '[v]']
    return _write_video(args + ['-frames:v', str(frames)] + _encoding(), out,
                        [source, gz, clock, rviz])


def _fit(text, factory, size, width):
    while size > 18:
        face = factory(size)
        if face.getlength(text) <= width:
            return face
        size -= 1
    face = factory(size)
    if face.getlength(text) > width:
        raise ValueError(f'Card text is too long: {text[:80]}')
    return face


def _wrap(text, face, width):
    lines = []
    for paragraph in text.splitlines():
        line = ''
        for word in paragraph.split():
            candidate = f'{line} {word}'.strip()
            if line and face.getlength(candidate) > width:
                lines.append(line)
                line = word
            else:
                line = candidate
        lines.append(line)
    return lines


def _load_results(path):
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    if not isinstance(data, dict) or not isinstance(data.get('title'), str) or not data['title'].strip():
        raise ValueError('Results JSON needs a nonempty string title')
    for key in ('subtitle', 'note'):
        if not isinstance(data.get(key, ''), str):
            raise ValueError(f'Results {key} must be a string')
    if not isinstance(data.get('rows'), list):
        raise ValueError('Results rows must be a list')
    for row in data['rows']:
        if (not isinstance(row, list) or len(row) not in (2, 3) or
                not isinstance(row[0], str) or
                any(not isinstance(cell, (str, int, float)) for cell in row)):
            raise ValueError('Each results row must contain [label, value, optional_value2]')
    return data


def render_results_card(results_json, out):
    data = _load_results(results_json)
    im, draw = v2._canvas()
    draw.text((150, 110), data['title'], fill=v2.FG,
              font=_fit(data['title'], lambda s: v2.font(s, 'B'), 64, 1620))
    y = 205
    for line in _wrap(data.get('subtitle', ''), v2.font(32, 'L'), 1620):
        draw.text((150, y), line, font=_fit(line, lambda s: v2.font(s, 'L'), 32, 1620), fill=v2.DIM)
        y += 42
    y += 40
    notes = _wrap(data.get('note', ''), v2.font(26), 1620)
    bottom = 970 - len(notes) * 36
    height = min(84, (bottom - y) / max(1, len(data['rows'])))
    if height < 40:
        raise ValueError('Too many rows or too much text for one results card')
    three = any(len(row) == 3 for row in data['rows'])
    columns = [(150, 750), (960, 360), (1390, 380)] if three else [(150, 1020), (1230, 540)]
    for row in data['rows']:
        draw.line([(150, y - 12), (1770, y - 12)], fill=(40, 45, 55), width=2)
        for index, (cell, (x, width)) in enumerate(zip(row, columns)):
            text = str(cell)
            factory = (lambda s: v2.mono(s, True)) if index else v2.font
            face = _fit(text, factory, min(38 if index else 34, int(height - 12)), width)
            draw.text((x + width - face.getlength(text) if index else x, y), text,
                      font=face, fill=v2.ACC if index else v2.FG)
        y += height
    for index, line in enumerate(notes):
        draw.text((150, 970 - len(notes) * 36 + index * 36), line,
                  font=_fit(line, v2.font, 26, 1620), fill=v2.DIM)
    with _destination(out, [results_json]) as temp:
        im.save(temp, format='PNG')
    return str(out)


def card_clip(png, out, seconds):
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('Card duration must be positive')
    frames = max(1, round(seconds * FPS))
    return _write_video(['-loop', '1', '-framerate', str(FPS)] + _input(png) + [
        '-vf', f'scale={W}:{H},setsar=1', '-frames:v', str(frames)] + _encoding(), out, [png])


def _title_card(data, out):
    im, draw = v2._canvas()
    draw.rectangle([150, 360, 158, 700], fill=v2.ACC)
    draw.text((190, 350), 'V3', font=v2.font(54, 'L'), fill=v2.ACC)
    draw.text((190, 430), data['title'], fill=v2.FG,
              font=_fit(data['title'], lambda s: v2.font(s, 'B'), 96, 1580))
    for index, line in enumerate(_wrap(data.get('subtitle', ''), v2.font(40, 'L'), 1570)):
        if index >= 6:
            raise ValueError('Title subtitle is too long for one card')
        draw.text((194, 550 + index * 50), line, fill=v2.FG,
                  font=_fit(line, lambda s: v2.font(s, 'L'), 40, 1570))
    draw.text((196, H - 110), 'Gazebo Harmonic  ·  RViz  ·  simulation time', font=v2.font(26), fill=v2.DIM)
    im.save(out)


def _concat(clips, out):
    with tempfile.TemporaryDirectory(prefix='video-v3-concat-') as tmp:
        listing = Path(tmp) / 'clips.txt'
        with listing.open('w', encoding='utf-8') as f:
            for clip in clips:
                escaped = str(Path(clip).resolve()).replace("'", "'\\''")
                f.write(f"file '{escaped}'\n")
        return _write_video(['-f', 'concat', '-safe', '0', '-i', str(listing),
                             '-map', '0:v:0', '-an', '-c', 'copy', '-movflags', '+faststart'], out, clips)


def demo(name, layout, results_json, out, raw_dir=None, capture_offset=0.0):
    data = _load_results(results_json)
    sources = list(_paths(name, raw_dir)) + [results_json]
    if Path(out).resolve() in (Path(p).resolve() for p in sources):
        raise ValueError('Demo output would overwrite an input')
    with tempfile.TemporaryDirectory(prefix='video-v3-demo-') as tmp:
        tmp = Path(tmp)
        _title_card(data, tmp / 'title.png')
        render_results_card(results_json, tmp / 'results.png')
        retime_rviz(name, raw_dir, capture_offset)
        compose(name, layout, tmp / 'body.mp4', raw_dir)
        card_clip(tmp / 'title.png', tmp / 'title.mp4', 3)
        card_clip(tmp / 'results.png', tmp / 'results.mp4', 5)
        return _concat([tmp / 'title.mp4', tmp / 'body.mp4', tmp / 'results.mp4'], out)


def reel(out, clips):
    if not clips:
        raise ValueError('Reel needs at least one clip')
    if Path(out).resolve() in (Path(p).resolve() for p in clips):
        raise ValueError('Reel output would overwrite an input')
    with tempfile.TemporaryDirectory(prefix='video-v3-reel-') as tmp:
        parts = []
        for index, clip in enumerate(clips):
            _, duration, _ = _probe(clip)
            frames = math.floor(duration * FPS + 1e-7)
            if frames < 1:
                raise ValueError(f'{clip}: less than one frame')
            duration = frames / FPS
            fade = min(.5, duration / 2)
            filters = (f'setpts=PTS-STARTPTS,fps={FPS},scale={W}:{H}:force_original_aspect_ratio=decrease,'
                       f'pad={W}:{H}:(ow-iw)/2:(oh-ih)/2:color=0x0e1015,setsar=1,'
                       f'fade=t=in:st=0:d={fade},fade=t=out:st={duration - fade}:d={fade}')
            part = Path(tmp) / f'{index}.mp4'
            _write_video(_input(clip) + ['-vf', filters, '-frames:v', str(frames)] + _encoding(), part, [clip])
            parts.append(part)
        return _concat(parts, out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    retime = commands.add_parser('v3-retime', help='retime RViz from wall time to sim time')
    retime.add_argument('name')
    composition = commands.add_parser('v3-compose', help='compose the two sim-time videos')
    composition.add_argument('name')
    composition.add_argument('layout', choices=LAYOUTS)
    composition.add_argument('out')
    card = commands.add_parser('v3-card', help='render results JSON to a PNG card')
    card.add_argument('results_json')
    card.add_argument('out')
    build = commands.add_parser('v3-demo', help='retime, compose, add 3 s title and 5 s results')
    build.add_argument('name')
    build.add_argument('layout', choices=LAYOUTS)
    build.add_argument('results_json')
    build.add_argument('out')
    montage = commands.add_parser('v3-reel', help='concatenate clips with 0.5 s fades through black')
    montage.add_argument('out')
    montage.add_argument('clips', nargs='+')
    for command in (retime, composition, build):
        command.add_argument('--raw-dir', help='recording directory (default: output/raw)')
    for command in (retime, build):
        command.add_argument('--capture-offset', type=float, default=0.0,
                             help='RViz capture start minus first clock wall sample, seconds (default: 0)')
    args = vars(parser.parse_args(argv))
    command = args.pop('command')
    try:
        action = {'v3-retime': retime_rviz, 'v3-compose': compose, 'v3-card': render_results_card,
                  'v3-demo': demo, 'v3-reel': reel}[command]
        print('wrote', action(**args))
    except (ValueError, OSError, RuntimeError, subprocess.CalledProcessError, KeyError, IndexError) as exc:
        parser.exit(1, f'video: {exc}\n')


if __name__ == '__main__':
    main()
