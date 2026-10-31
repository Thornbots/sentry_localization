# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.
"""Optional full corrected-field reproduction; explicitly supply FIELD_MAP_WORLD."""
import importlib.util
import os
from pathlib import Path
import unittest

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('field_map', ROOT / 'tools/field_map.py')
field_map = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(field_map)


@unittest.skipUnless(os.environ.get('FIELD_MAP_WORLD'), 'FIELD_MAP_WORLD fixture not supplied')
class ActualFieldTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.triangles = field_map.load_world_triangles(Path(os.environ['FIELD_MAP_WORLD']))

    def test_world_frame(self):
        low = self.triangles.min(axis=(0, 1))
        high = self.triangles.max(axis=(0, 1))
        np.testing.assert_allclose(low, [-6.168884277, -4.168884277, -.025], atol=1e-6)
        np.testing.assert_allclose(high, [6.168884277, 4.168884277, .498611624], atol=1e-6)
        self.assertEqual(len(self.triangles), 8562)

    def test_saved_map_reproduces(self):
        meta = yaml.safe_load((ROOT / 'map/clean_map.yaml').read_text())
        field_map.validate_metadata(meta)
        with Image.open(ROOT / 'map' / meta['image']) as image:
            saved = np.asarray(image)
        generated = field_map.rasterize(self.triangles, .357296694, meta['resolution'],
                                        meta['origin'][:2], saved.shape, grow=1)
        np.testing.assert_array_equal(saved, generated)


if __name__ == '__main__':
    unittest.main()
