#!/usr/bin/env python3
"""Summarise a perception JSONL log: per-pipeline averages + confusion matrix (rows = truth, cols = predicted)."""
import json
import sys

LABELS = ['ground', 'static', 'dynamic', 'dust', 'steep']
CATS = ['ground', 'obstacle', 'dust', 'overhang']


def main(path):
    rows = [json.loads(line) for line in open(path)]
    out = {}
    for topic in ('/perception_naive/metrics', '/perception/metrics'):
        r = [x['data'] for x in rows if x['topic'] == topic]
        if not r:
            continue
        last = r[-1]
        out[topic] = last
        print(f"\n{topic}  ({last['frames']} frames)")
        for k in ('avg_ms', 'avg_false_obstacle_pts', 'avg_phantom_cells', 'avg_dust_recall_dusty',
                  'avg_ground_recall', 'avg_obstacle_recall'):
            print(f'  {k:26s} {last[k]:10.3f}')
        print('  confusion (row-normalised, %):  ' + '  '.join(f'{c:>8s}' for c in CATS))
        for lab, row in zip(LABELS, last['confusion']):
            s = sum(row) or 1
            print(f'  {lab:10s} n={s:9d}           ' + '  '.join(f'{100.0 * v / s:8.1f}' for v in row))
    return out


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'output/logs/perception.jsonl')
