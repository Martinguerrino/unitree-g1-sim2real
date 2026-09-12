"""One application for two isolated DDS-backed telemetry sources."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

from .config import load
from .dds import DDSRobotInterface, initialize


def revision(repository):
    result = subprocess.run(['git', '-C', str(repository), 'rev-parse', 'HEAD'], capture_output=True, text=True)
    return result.stdout.strip() if result.returncode == 0 else None


def stop_process(process):
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def main(root: Path):
    parser = argparse.ArgumentParser(description='Read G1 body joints over Unitree DDS from a paused simulator.')
    parser.add_argument('--sim', choices=['mujoco', 'isaac', 'real'], required=True)
    parser.add_argument('--config', type=Path, help='Optional YAML override')
    parser.add_argument('--backend-python', help='Simulator Python executable or Isaac python.sh')
    parser.add_argument('--check', action='store_true', help='Validate configuration and files; does not run DDS/physics')
    args = parser.parse_args()
    if args.sim == 'real':
        parser.error('Real G1 is not implemented; this MVP only supports simulation telemetry')
    process = None
    robot = None
    run_dir = None
    result = {'schema_version': 1, 'experiment': 'dds_joint_telemetry', 'sim': args.sim,
              'success': False, 'samples_received': 0, 'physics_benchmark': False,
              'valid_fields': ['q', 'dq'], 'error': None}
    started = time.monotonic()
    try:
        config = load(root, args.sim, args.config)
        executable = args.backend_python or config['backend']['python']
        if executable is None and args.sim == 'mujoco':
            executable = sys.executable
        if executable:
            executable = shutil.which(executable)
            if not executable:
                raise ValueError('Backend Python executable not found')
        repository = root / config['backend']['repository']
        if not (repository / config['backend']['scene']).is_file():
            raise FileNotFoundError('External scene missing; run git submodule update --init --recursive')
        if args.sim == 'mujoco':
            for key in ('model', 'meshes'):
                if not (repository / config['backend'][key]).exists():
                    raise FileNotFoundError(f'MuJoCo {key} is missing')
        config['backend']['python'] = executable
        result['external_commit'] = revision(repository)
        if args.check:
            print(json.dumps({'configuration_valid': True, 'runtime_validated': False,
                              'backend_python': executable, 'external_commit': result['external_commit']}, indent=2))
            return 0
        if not executable:
            raise ValueError('Isaac requires --backend-python /path/to/isaac-sim/python.sh')
        if importlib.util.find_spec('unitree_sdk2py') is None:
            raise RuntimeError('Unitree SDK2 is missing in core Python; see README installation')
        run_dir = root / config['output'] / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:8])
        run_dir.mkdir(parents=True)
        config_path = run_dir / 'config.json'
        config_path.write_text(json.dumps(config, indent=2))
        initialize(config['dds'])
        robot = DDSRobotInterface(config['dds'], config['joints'])
        with (run_dir / 'backend.log').open('w') as log:
            process = subprocess.Popen([executable, str(root / 'scripts/bridge.py'), '--sim', args.sim,
                                        '--config', str(config_path)], cwd=root, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            print(f'Telemetría DDS ({args.sim}); log: {run_dir / "backend.log"}', flush=True)
            with (run_dir / 'states.jsonl').open('w') as states:
                deadline = time.monotonic() + config['telemetry']['startup_timeout_s']
                while result['samples_received'] < config['telemetry']['samples']:
                    if process.poll() is not None:
                        raise RuntimeError(f'Backend exited with code {process.returncode}; see backend.log')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Timed out waiting for fresh DDS state; see backend.log')
                    try:
                        state = robot.read_state(min(0.25, deadline - time.monotonic()))
                    except TimeoutError:
                        continue
                    states.write(json.dumps(asdict(state), allow_nan=False) + '\n')
                    result['samples_received'] += 1
                    deadline = time.monotonic() + config['telemetry']['stale_timeout_s']
        result['success'] = True
    except (Exception, KeyboardInterrupt) as error:
        result['error'] = str(error) or type(error).__name__
    finally:
        stop_process(process)
        if robot is not None:
            robot.close()
        result['wall_duration_s'] = time.monotonic() - started
        if run_dir is not None:
            (run_dir / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1
