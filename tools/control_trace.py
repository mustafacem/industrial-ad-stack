#!/usr/bin/env python3
"""Time-aligned trace of a mission/control JSONL log (sim debug + control metrics) every N seconds."""
import bisect
import json
import sys

rows = [json.loads(line) for line in open(sys.argv[1])]
step = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
c = [r['data'] for r in rows if r['topic'] == '/control/metrics']
d = [r['data'] for r in rows if r['topic'] == '/sim/debug']
ct = [x['t'] - 100.0 for x in c]
t = 0.0
while t <= d[-1]['t']:
    y = min(d, key=lambda r: abs(r['t'] - t))
    x = c[min(len(c) - 1, bisect.bisect_left(ct, t))]
    print('t %5.1f %-8s seg %d/%d lat %+.2f head %+5.1f | v %+.2f vdes %+.2f a %+.2f | pose (%6.1f, %5.1f, %6.1f deg) steer %+.2f'
          % (t, x['state'], x['segment'], x['segments'], x['lat_err'], x['head_err_deg'], y['v'], x['v_des'], x['a_cmd'],
             y['x'], y['y'], y['yaw'] * 57.2958, y['steer']))
    t += step
print('final', {k: round(c[-1][k], 3) for k in ('lat_rms', 'lat_max', 'head_rms_deg')}, 'pose', round(d[-1]['x'], 2), round(d[-1]['y'], 2), round(d[-1]['yaw'] * 57.2958, 1))
