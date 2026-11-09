import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('voo', Path(__file__).parents[1] / 'voo_autonomo.py')
voo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(voo)


class PlanTests(unittest.TestCase):
    def setUp(self):
        self.p = dict(FENCE_ENABLE=1, FENCE_TYPE=3, FENCE_ACTION=1,
                      FENCE_ALT_MAX=7, FENCE_MARGIN=2, SIM_RC_FAIL=0,
                      FS_GCS_ENABLE=1, FS_THR_ENABLE=1, ARMING_CHECK=1)

    def test_valid_plan(self):
        voo.validate_plan(self.p, 5, 2)

    def test_reject_unsafe_parameters(self):
        for key, value in [('FENCE_ENABLE', 0), ('FENCE_TYPE', 2),
                           ('FENCE_ACTION', 0), ('FENCE_ALT_MAX', 6),
                           ('SIM_RC_FAIL', 1), ('SIM_RC_FAIL', None),
                           ('FS_GCS_ENABLE', 0), ('FS_THR_ENABLE', 0),
                           ('ARMING_CHECK', 0)]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                voo.validate_plan(dict(self.p, **{key: value}), 5, 2)

    def test_reject_invalid_altitudes(self):
        for alt in [0, 7, 10, float('nan'), float('inf')]:
            with self.subTest(alt=alt), self.assertRaises(ValueError):
                voo.validate_plan(self.p, alt, 2)

    def test_reject_invalid_distances(self):
        for distance in [0, -1, 6, float('nan'), float('inf')]:
            with self.subTest(distance=distance), self.assertRaises(ValueError):
                voo.validate_plan(self.p, 5, distance)

    def test_parameter_id_normalization(self):
        self.assertEqual(voo.normalize_id(b'FENCE_ENABLE\0'), 'FENCE_ENABLE')
        self.assertEqual(voo.normalize_id('FENCE_ENABLE\0'), 'FENCE_ENABLE')

    def test_modern_arming_checks(self):
        del self.p['ARMING_CHECK']
        self.p['ARMING_SKIPCHK'] = 0
        voo.validate_plan(self.p, 5, 2)
        self.p['ARMING_SKIPCHK'] = 8
        with self.assertRaises(ValueError):
            voo.validate_plan(self.p, 5, 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
