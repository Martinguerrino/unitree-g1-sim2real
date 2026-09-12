#!/usr/bin/env python3
"""Summarize telemetry runs separately per backend, never as grasp success."""
import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='?', default=Path(__file__).resolve().parents[1] / 'experiments/runs')
    args = parser.parse_args()
    groups = {}
    for path in args.runs.glob('*/result.json'):
        run = json.loads(path.read_text())
        if run.get('experiment') != 'dds_joint_telemetry':
            continue
        group = groups.setdefault(run['sim'], {'runs': 0, 'telemetry_successes': 0})
        group['runs'] += 1
        group['telemetry_successes'] += int(run['success'])
    for group in groups.values():
        group['telemetry_success_rate'] = group['telemetry_successes'] / group['runs']
    print(json.dumps(groups, indent=2))


if __name__ == '__main__':
    main()
