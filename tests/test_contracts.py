import math
from pathlib import Path
import queue
from types import SimpleNamespace
import unittest

from g1_sim2real.config import joint_indices, load, merge, validate
from g1_sim2real.dds import DDSRobotInterface, decode

ROOT = Path(__file__).resolve().parents[1]


class ContractsTest(unittest.TestCase):
    def test_backend_configs_share_motor_contract(self):
        mujoco = load(ROOT, 'mujoco')
        isaac = load(ROOT, 'isaac')
        self.assertEqual(mujoco['joints'], isaac['joints'])
        self.assertEqual(mujoco['dds'], isaac['dds'])
        self.assertEqual(mujoco['telemetry'], isaac['telemetry'])

    def test_nested_override_keeps_other_settings(self):
        base = {'dds': {'domain': 42, 'interface': 'lo'}}
        result = merge(base, {'dds': {'domain': 43}})
        self.assertEqual(result['dds'], {'domain': 43, 'interface': 'lo'})
        self.assertEqual(base['dds']['domain'], 42)

    def test_joint_mapping_does_not_assume_simulator_order(self):
        self.assertEqual(joint_indices(['hand', 'b', 'a'], ['a', 'b']), [2, 1])
        with self.assertRaises(ValueError):
            joint_indices(['a'], ['a', 'b'])
        with self.assertRaises(ValueError):
            joint_indices(['a', 'a'], ['a'])

    def test_invalid_config_rejected(self):
        for value in (0, -1, math.nan, True):
            config = load(ROOT, 'mujoco')
            config['telemetry']['frequency_hz'] = value
            with self.assertRaises(ValueError):
                validate(config)
        config = load(ROOT, 'mujoco')
        config['dds']['interface'] = 'eth0'
        with self.assertRaises(ValueError):
            validate(config)

    def test_nonfinite_and_short_state_rejected(self):
        for motors in ([], [SimpleNamespace(q=math.nan, dq=0)]):
            with self.assertRaises(ValueError):
                decode(SimpleNamespace(motor_state=motors, tick=0), ['joint'])

    def test_stale_tick_times_out_and_latest_sample_wins(self):
        robot = DDSRobotInterface.__new__(DDSRobotInterface)
        robot.joints = ['joint']
        robot.messages = queue.Queue(maxsize=1)
        robot.last_tick = None
        def message(tick):
            return SimpleNamespace(motor_state=[SimpleNamespace(q=1, dq=2)], tick=tick)
        robot._receive(message(0))
        robot._receive(message(1))
        self.assertEqual(robot.read_state(.05).tick, 1)
        robot._receive(message(1))
        with self.assertRaises(TimeoutError):
            robot.read_state(.01)
        robot._receive(message(2))
        self.assertEqual(robot.read_state(.05).tick, 2)

    def test_mujoco_external_model_loads_without_modification(self):
        try:
            import mujoco
        except ImportError:
            self.skipTest('Optional MuJoCo runtime is not installed')
        from g1_sim2real.backends import mujoco_state
        config = load(ROOT, 'mujoco')
        model = ROOT / config['backend']['repository'] / config['backend']['model']
        if not model.exists():
            self.skipTest('External MuJoCo submodule is not initialized')
        before = model.read_bytes()
        with mujoco_state(ROOT, config) as read:
            q, dq = read()
            self.assertEqual(len(q), 29)
            self.assertEqual(len(dq), 29)
            self.assertTrue(all(math.isfinite(float(x)) for x in list(q) + list(dq)))
        self.assertEqual(before, model.read_bytes())


if __name__ == '__main__':
    unittest.main()
