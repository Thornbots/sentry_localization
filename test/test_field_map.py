# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.
"""Standalone CAD raster regressions and independent saved-map decoding."""
import importlib.util
from itertools import permutations
from pathlib import Path
import struct
import tempfile
import unittest

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('field_map', ROOT / 'tools/field_map.py')
field_map = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(field_map)


def decode_saved(name):
    """Decode saved grayscale using YAML/Nav2 trinary rules, independently of generator."""
    meta = yaml.safe_load((ROOT / f'map/{name}.yaml').read_text())
    with Image.open(ROOT / 'map' / meta['image']) as image:
        if image.mode != 'L':
            raise AssertionError('expected grayscale map')
        pixels = np.asarray(image)
    if meta.get('mode', 'trinary') != 'trinary' or meta['negate'] != 0:
        raise AssertionError('expected current trinary, negate=0 maps')
    probability = (255 - pixels.astype(float)) / 255
    decoded = np.full(pixels.shape, -1, dtype=np.int8)
    decoded[probability >= meta['occupied_thresh']] = 100
    decoded[probability <= meta['free_thresh']] = 0
    return meta, pixels, decoded


def pixel_at(grid, meta, x, y):
    col, row = np.floor((np.array([x, y]) - meta['origin'][:2]) /
                        meta['resolution']).astype(int)
    return np.flipud(grid)[row, col]


class SliceTest(unittest.TestCase):
    def assert_permutations(self, triangle, z=0):
        expected = field_map.slice_cells(np.array([triangle]), z, .1, [0, 0], (40, 40))
        for order in permutations(range(3)):
            actual = field_map.slice_cells(np.array([np.array(triangle)[list(order)]]),
                                           z, .1, [0, 0], (40, 40))
            np.testing.assert_array_equal(actual, expected)
        return expected

    def test_review_exact_vertex_repro_all_orders(self):
        triangle = [[1., 1., -1.], [1., 2., 0.], [3., 1., 1.]]
        segments = field_map.slice_segments(triangle, 0)
        np.testing.assert_allclose(segments, [[[1, 2], [2, 1]]], atol=1e-12)
        self.assertGreater(self.assert_permutations(triangle).sum(), 10)

    def test_exact_coplanar_edge(self):
        triangle = [[.15, .15, 0], [1.15, .15, 0], [.15, .95, 1]]
        np.testing.assert_allclose(field_map.slice_segments(triangle, 0),
                                   [[[.15, .15], [1.15, .15]]], atol=1e-12)
        grid = self.assert_permutations(triangle)
        self.assertTrue(grid[1, 1:12].all())
        self.assertEqual(grid.sum(), 11)

    def test_coplanar_face_edges_only(self):
        triangle = [[.15, .15, 0], [1.15, .15, 0], [.15, .95, 0]]
        segments = field_map.slice_segments(triangle, 0)
        self.assertEqual(len(segments), 3)
        lengths = sorted(np.linalg.norm(q-p) for p, q in segments)
        np.testing.assert_allclose(lengths, [.8, 1., np.hypot(1, .8)], atol=1e-12)
        grid = self.assert_permutations(triangle)
        self.assertTrue(grid[1, 6])
        self.assertTrue(grid[4, 1])
        self.assertFalse(grid[4, 4])

    def test_tangent_vertex_and_duplicate_vertices_emit_no_segment(self):
        for triangle in [[[1, 2, 0], [0, 0, 1], [3, 1, 1]],
                         [[1, 2, 0], [1, 2, 0], [3, 1, 1]]]:
            self.assertEqual(field_map.slice_segments(triangle, 0), [])
            self.assertFalse(self.assert_permutations(triangle).any())

    def test_plane_perturbations_inside_and_outside_tolerance(self):
        triangle = np.array([[1., 1., -1.], [1., 2., 0.], [3., 1., 1.]])
        self.assert_permutations(triangle)
        tolerance = field_map.SLICE_TOLERANCE_M
        shifted = triangle + [.017, .031, 0]
        shifted_exact = self.assert_permutations(shifted)
        for delta in [-tolerance/4, tolerance/4]:
            np.testing.assert_allclose(field_map.slice_segments(triangle, delta),
                                       [[[1, 2], [2, 1]]], atol=tolerance)
            self.assertGreater(self.assert_permutations(triangle, delta).sum(), 10)
            np.testing.assert_array_equal(self.assert_permutations(shifted, delta), shifted_exact)
        for delta in [-1e-5, 1e-5]:
            segments = field_map.slice_segments(triangle, delta)
            np.testing.assert_allclose(segments, [[[1, 2], [2, 1]]], atol=3e-5)
            self.assertGreater(self.assert_permutations(triangle, delta).sum(), 10)

    def test_near_coincident_endpoints_deduplicate(self):
        tolerance = field_map.SLICE_TOLERANCE_M
        triangle = [[1, 2, 0], [1+tolerance/4, 2, 0], [3, 1, 1]]
        self.assertEqual(field_map.slice_segments(triangle, 0), [])
        self.assertFalse(self.assert_permutations(triangle).any())

    def test_invalid_tolerance(self):
        for tolerance in [0, -1, float('nan')]:
            with self.assertRaises(ValueError):
                field_map.slice_segments(np.zeros((3, 3)), 0, tolerance)


class FieldMapTest(unittest.TestCase):
    def test_saved_image_decodes_s2_lidar_slice_and_unknown_regions(self):
        meta, pixels, decoded = decode_saved('clean_map')
        self.assertEqual(meta['resolution'], .02)
        self.assertEqual(meta['origin'], [-6.230545, -4.249455, 0.0])
        self.assertEqual(pixels.shape, (424, 624))
        values, counts = np.unique(decoded, return_counts=True)
        self.assertEqual(dict(zip(values, counts)), {-1: 27928, 0: 227149, 100: 9499})
        self.assertEqual(set(np.unique(pixels)), {0, 128, 254})
        self.assertTrue(np.all(decoded[pixels == 0] == 100))
        self.assertTrue(np.all(decoded[pixels == 128] == -1))
        self.assertTrue(np.all(decoded[pixels == 254] == 0))
        # (x, y, cell) at the 0.357 m slice; README "Saved map" lists what it sees.
        for x, y, cell in [(.08, 0, 100), (0, 0, -1), (.5, 0, 0),  # Central Wall base
                           (0, 1.6, 0), (0, 1.98, 100),  # under the Limbo Bar, its post
                           (-3, -2.5, 100), (3, -2.5, 100), (-3.5, -2.5, -1),  # 0.41 m tiers
                           (-3.5, -1.5, 0), (0, 3, 0),  # 0.21 m platforms
                           (-3.5, -3.25, 100), (-3.5, -3.5, 0),  # Side Limbo spacer, bars
                           (-4, -.635, 0), (-5.25, -3, 0), (0, 4.2, -1)]:
            self.assertEqual(pixel_at(decoded, meta, x, y), cell, (x, y))

    def test_metadata_rejects_incompatible_modes_negate_and_thresholds(self):
        meta = yaml.safe_load((ROOT / 'map/clean_map.yaml').read_text())
        field_map.validate_metadata(meta)
        field_map.validate_metadata({**meta, 'mode': 'trinary'})
        variants = [{'mode': 'scale'}, {'mode': 'raw'}, {'negate': 1},
                    {'free_thresh': .5}, {'occupied_thresh': .4},
                    {'free_thresh': 127/255}, {'occupied_thresh': 127/255},
                    {'free_thresh': 0}, {'occupied_thresh': 1},
                    {'occupied_thresh': float('nan')}, {'resolution': 0},
                    {'origin': [0, 0, .1]}]
        for variant in variants:
            with self.subTest(variant=variant), self.assertRaises(ValueError):
                field_map.validate_metadata({**meta, **variant})
        implicit = dict(meta)
        implicit.pop('mode', None)
        field_map.validate_metadata(implicit)

    def test_synthetic_enclosure_fills_interior_and_keeps_exterior_unknown(self):
        triangles = []
        corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        for i, (x, y) in enumerate(corners):
            u, v = corners[(i+1) % 4]
            triangles.extend([[[x, y, -1], [u, v, -1], [u, v, 1]],
                              [[x, y, -1], [u, v, 1], [x, y, 1]]])
        image = np.flipud(field_map.rasterize(np.array(triangles), 0, .1, [-2, -2], (40, 40)))
        self.assertEqual(image[20, 20], 254)
        self.assertEqual(image[20, 30], 0)
        self.assertEqual(image[20, 35], 128)

    def test_seed_selects_flood_fill_start(self):
        triangles = []
        corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
        for i, (x, y) in enumerate(corners):
            u, v = corners[(i+1) % 4]
            triangles.extend([[[x, y, -1], [u, v, -1], [u, v, 1]],
                              [[x, y, -1], [u, v, 1], [x, y, 1]]])
        triangles = np.array(triangles) + [3, 0, 0]  # enclosure no longer covers (0, 0)
        with self.assertRaisesRegex(ValueError, 'leaks'):
            field_map.rasterize(triangles, 0, .1, [-2, -2], (40, 70))
        image = np.flipud(field_map.rasterize(triangles, 0, .1, [-2, -2], (40, 70), seed=(3, 0)))
        self.assertEqual(image[20, 50], 254)
        self.assertEqual(image[20, 20], 128)
        with self.assertRaisesRegex(ValueError, 'seed'):
            field_map.rasterize(triangles, 0, .1, [-2, -2], (40, 70), seed=(4, 0))

    def test_leaking_geometry_rejected(self):
        with self.assertRaisesRegex(ValueError, 'leaks'):
            field_map.rasterize(np.zeros((1, 3, 3)), .35, .1, [-1, -1], (20, 20))

    def test_binary_stl_and_sdf_fixture_loads_scale_and_parent_transforms(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'world').mkdir()
            vertices = [0., 0., 0., 1., 0., 0., 0., 1., 0.]
            (root / 'triangle.stl').write_bytes(b'\0'*80 + struct.pack('<I', 1) +
                                               struct.pack('<12fH', 0., 0., 1., *vertices, 0))
            world = root / 'world/fixture.sdf'
            world.write_text('<sdf><world name="field"><model name="any_field">'
                             '<pose>1 2 0 0 0 1.5707963267948966</pose>'
                             '<link><pose>1 0 0 0 0 0</pose><collision>'
                             '<pose>0 0 .5 0 0 0</pose><geometry><mesh>'
                             '<uri>model://sim/triangle.stl</uri><scale>2 3 1</scale>'
                             '</mesh></geometry></collision></link></model></world></sdf>')
            np.testing.assert_allclose(field_map.load_world_triangles(world),
                                       [[[1, 3, .5], [1, 5, .5], [-2, 3, .5]]], atol=1e-12)
            world.write_text('<sdf><world name="field"></world></sdf>')
            with self.assertRaisesRegex(ValueError, 'no field model'):
                field_map.load_world_triangles(world)

    def test_invalid_stl_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'truncated.stl'
            path.write_bytes(b'bad')
            with self.assertRaisesRegex(ValueError, 'truncated'):
                field_map.load_stl(path)

    def test_relative_pose_rejected(self):
        element = field_map.ET.fromstring('<link><pose relative_to="other">0 0 0 0 0 0</pose></link>')
        with self.assertRaisesRegex(ValueError, 'relative_to'):
            field_map.pose(element)


if __name__ == '__main__':
    unittest.main()
