import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from navigation_scenario import load_scenario, pose_errors


class ScenarioTests(unittest.TestCase):
    def setUp(self):
        self.data = json.loads(Path(__file__).with_name('waypoints.json').read_text())

    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'scenario.json'
            path.write_text(json.dumps(data))
            return load_scenario(path)

    def test_example(self):
        self.assertEqual(len(self.load(self.data)['goals']), 4)

    def test_angle_wrap(self):
        distance, angle = pose_errors({'x': 2, 'y': 3, 'yaw_deg': 179}, 2, 3, math.radians(-179))
        self.assertEqual(distance, 0)
        self.assertAlmostEqual(angle, 2)

    def test_reject_invalid_scenarios(self):
        for key, value in [('x', math.nan), ('timeout', 0), ('position_tolerance', -1), ('yaw_deg', True)]:
            with self.subTest(key=key):
                data = copy.deepcopy(self.data)
                data['goals'][0][key] = value
                with self.assertRaises(ValueError):
                    self.load(data)

    def test_reject_fault_after_deadline(self):
        self.data['goals'][1]['faults'][0]['duration'] = 200
        with self.assertRaises(ValueError):
            self.load(self.data)

    def test_reject_unimplemented_fault(self):
        self.data['goals'][1]['faults'][0]['id'] = 'typo'
        with self.assertRaises(ValueError):
            self.load(self.data)

    def test_reject_unity_frame(self):
        self.data['frame_id'] = 'unity'
        with self.assertRaises(ValueError):
            self.load(self.data)


if __name__ == '__main__':
    unittest.main()
