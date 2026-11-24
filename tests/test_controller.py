"""Pure control-logic tests with a minimal Sofa import stub; no FEM engine."""

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sofa = ModuleType('Sofa')
core = ModuleType('Sofa.Core')
core.Controller = object
sim = ModuleType('Sofa.Simulation')
constants = ModuleType('Sofa.constants')
constants.Key = SimpleNamespace(space=' ')
sofa.Core, sofa.Simulation = core, sim
spec = importlib.util.spec_from_file_location('tested_tongue_motion', ROOT / 'tongue_motion.py')
scene = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'Sofa': sofa, 'Sofa.Core': core, 'Sofa.Simulation': sim, 'Sofa.constants': constants}):
    spec.loader.exec_module(scene)


def controller():
    obj = scene.TongueController.__new__(scene.TongueController)
    for key, value in dict(manualLeftRight=0., manualUpDown=0., manualForwardBack=0.,
                           automaticMotion=False, keyboardEventCount=0, automaticCyclePhase=0.).items():
        setattr(obj, key, SimpleNamespace(value=value))
    obj._apply_command = lambda command: setattr(obj, 'current_command', list(command))
    return obj


class ControllerTests(unittest.TestCase):
    def test_coordinate_convention(self):
        self.assertEqual(scene.TongueController._to_model_coordinates([1, 2, 3]), [1, -2, -3])

    def test_up_down_are_opposite(self):
        obj = controller()
        obj.onKeypressedEvent({'key': 'w'})
        self.assertAlmostEqual(obj.manualUpDown.value, .08)
        obj.onKeypressedEvent({'key': 's'})
        self.assertAlmostEqual(obj.manualUpDown.value, 0.)

    def test_manual_range_is_clamped(self):
        obj = controller()
        for _ in range(10):
            obj.onKeypressedEvent({'key': 'D'})
        self.assertEqual(obj.manualLeftRight.value, .55)

    def test_reset_command(self):
        obj = controller()
        obj.onKeypressedEvent({'key': 'E'})
        obj.onKeypressedEvent({'key': '0'})
        self.assertEqual(obj._manual_command(), [0., 0., 0.])

    def test_switching_to_manual_zeros_target(self):
        obj = controller()
        obj.automaticMotion.value = True
        obj.manualUpDown.value = .3
        obj.onKeypressedEvent({'key': ' '})
        self.assertFalse(obj.automaticMotion.value)
        self.assertEqual(obj._manual_command(), [0., 0., 0.])

    def test_automatic_cycle_is_periodic(self):
        obj = controller()
        a = obj._automatic_command(.9)
        b = obj._automatic_command(.9 + obj.CYCLE_SECONDS)
        for left, right in zip(a, b):
            self.assertAlmostEqual(left, right)

    def test_automatic_cycle_starts_at_neutral(self):
        obj = controller()
        self.assertEqual(obj._automatic_command(0), [0., 0., 0.])

    def test_automatic_loop_has_no_end_discontinuity(self):
        obj = controller()
        a = obj._automatic_command(obj.CYCLE_SECONDS - 1e-6)
        b = obj._automatic_command(1e-6)
        self.assertLess(math.dist(a, b), 1e-5)


if __name__ == '__main__':
    unittest.main()
