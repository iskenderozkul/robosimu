import unittest

import numpy as np

from robosimu.model import load_model, model_from_dict, validate_data


class TestModel(unittest.TestCase):
    def test_load(self):
        m = load_model("models/six_axis_arm.json")
        self.assertEqual(m.n, 6)
        self.assertGreater(m.summary()["total_mass_kg"], 10)
        self.assertEqual(m.links[0].motor.type, "bldc")

    def test_payload_increases_mass(self):
        m = load_model("models/six_axis_arm.json")
        mp = m.with_payload(3.0)
        self.assertAlmostEqual(mp.links[-1].mass, m.links[-1].mass + 3.0)
        self.assertAlmostEqual(mp.summary()["total_mass_kg"], m.summary()["total_mass_kg"] + 3.0)

    def test_validation_catches_missing(self):
        data = {"links": [{"a": 0.0}]}
        self.assertTrue(validate_data(data))

    def test_validation_ok(self):
        data = __import__("json").loads(open("models/six_axis_arm.json", encoding="utf-8").read())
        self.assertEqual(validate_data(data), [])


if __name__ == "__main__":
    unittest.main()
