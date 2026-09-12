#!/usr/bin/env python3
"""Child-process entry point, intentionally separate from the application."""
import argparse
import json
import importlib.metadata
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from g1_sim2real.backends import isaac_state, mujoco_state
from g1_sim2real.dds import initialize, StatePublisher


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim', choices=['mujoco', 'isaac'], required=True)
    parser.add_argument('--config', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    print(json.dumps({'mode': 'paused_joint_telemetry', 'python': sys.version,
                      'sdk2_version': importlib.metadata.version('unitree_sdk2py'),
                      'sim': args.sim}), flush=True)
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    adapter = {'mujoco': mujoco_state, 'isaac': isaac_state}[args.sim]
    with adapter(ROOT, config) as read:
        initialize(config['dds'])
        publisher = StatePublisher(config['dds'])
        try:
            tick = 0
            while running:
                started = time.monotonic()
                q, dq = read()
                publisher.write(q, dq, tick)
                tick += 1
                time.sleep(max(0, 1 / config['telemetry']['frequency_hz'] - (time.monotonic() - started)))
        finally:
            publisher.close()


if __name__ == '__main__':
    main()
