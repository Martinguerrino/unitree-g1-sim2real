"""Shared YAML configuration, with backend and optional local overrides."""
from copy import deepcopy
from pathlib import Path
import math
import yaml


def merge(base, override):
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


def load(root: Path, sim: str, override: Path | None = None):
    config = {}
    paths = [root / 'configs/common.yaml', root / f'configs/{sim}.yaml']
    if override:
        paths.append(override)
    for path in paths:
        document = yaml.safe_load(path.read_text())
        if not isinstance(document, dict):
            raise ValueError(f'{path}: expected a YAML mapping')
        config = merge(config, document)
    validate(config)
    return config


def validate(config):
    joints = config['joints']
    if len(joints) != 29 or len(set(joints)) != 29:
        raise ValueError('Expected 29 unique G1 body joint names')
    for key in ('frequency_hz', 'samples', 'startup_timeout_s', 'stale_timeout_s'):
        value = config['telemetry'][key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'telemetry.{key} must be finite and positive')
    if not isinstance(config['telemetry']['samples'], int):
        raise ValueError('telemetry.samples must be an integer')
    domain = config['dds']['domain']
    if isinstance(domain, bool) or not isinstance(domain, int) or not 1 <= domain <= 232:
        raise ValueError('Simulation DDS domain must be an integer in 1..232')
    if config['dds']['interface'] != 'lo':
        raise ValueError('This simulation-only MVP requires DDS interface lo')


def joint_indices(actual, expected):
    actual = list(actual)
    if len(actual) != len(set(actual)):
        raise ValueError('Backend contains duplicate joint names')
    missing = set(expected) - set(actual)
    if missing:
        raise ValueError(f'Backend is missing joints: {sorted(missing)}')
    return [actual.index(name) for name in expected]
