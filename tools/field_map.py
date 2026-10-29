#!/usr/bin/env python3
# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.
"""Rasterize SDF field collision meshes at the nominal lidar height."""
import argparse
from itertools import combinations
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml


OCCUPIED, UNKNOWN, FREE = 0, 128, 254
SLICE_TOLERANCE_M = 1e-7


def validate_metadata(meta):
    """Require trinary, unrotated metadata that decodes our three intensities safely."""
    if meta.get('mode', 'trinary') != 'trinary' or meta.get('negate', 0) != 0:
        raise ValueError('expected trinary, non-negated map metadata')
    origin = np.asarray(meta['origin'], dtype=float)
    res = float(meta['resolution'])
    free, occupied = float(meta['free_thresh']), float(meta['occupied_thresh'])
    if origin.shape != (3,) or not np.isfinite(origin).all() or origin[2] != 0:
        raise ValueError('expected finite, unrotated map origin')
    if not np.isfinite([res, free, occupied]).all() or res <= 0 or not 0 <= free < occupied <= 1:
        raise ValueError('invalid resolution or occupancy thresholds')
    p_occ, p_unknown, p_free = 1 - np.array([OCCUPIED, UNKNOWN, FREE]) / 255.0
    if not (p_free < free < p_unknown < occupied < p_occ):
        raise ValueError('map thresholds do not safely decode occupied/unknown/free intensities')


def pose(element):
    """SDF parent-relative xyz/rpy to a homogeneous transform."""
    value = element.find('pose')
    if value is not None and value.get('relative_to'):
        raise ValueError('relative_to poses are unsupported')
    xyzrpy = np.fromstring(value.text, sep=' ') if value is not None else np.zeros(6)
    if len(xyzrpy) != 6 or not np.isfinite(xyzrpy).all():
        raise ValueError('expected finite xyz/rpy pose')
    x, y, z, roll, pitch, yaw = xyzrpy
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    out = np.eye(4)
    out[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                   [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    out[:3, 3] = [x, y, z]
    return out


def load_stl(path):
    """Read binary STL vertices, checking the declared byte count."""
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError('truncated STL')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count or count == 0:
        raise ValueError('expected nonempty binary STL')
    dtype = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    vertices = np.frombuffer(data, dtype=dtype, count=count, offset=84)['v'].astype(float)
    if not np.isfinite(vertices).all():
        raise ValueError('nonfinite STL')
    return vertices


def load_world_triangles(world):
    """Load the same field collisions and parent transforms as combat::load_field."""
    world = Path(world).resolve()
    model = ET.parse(world).find("./world/model[@name='ARCC_Field_2026']")
    if model is None:
        raise ValueError('missing ARCC_Field_2026 model')
    result = []
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            mesh = collision.find('./geometry/mesh')
            if mesh is None:
                raise ValueError('expected mesh collision')
            uri = mesh.findtext('uri', '')
            if not uri.startswith('model://sim/'):
                raise ValueError('expected sim mesh URI')
            path = (world.parent.parent / uri[len('model://sim/'):]).resolve()
            if not path.is_relative_to(world.parent.parent):
                raise ValueError('mesh escapes sim package')
            scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
            if len(scale) != 3 or not np.isfinite(scale).all() or (scale <= 0).any():
                raise ValueError('invalid mesh scale')
            transform = pose(model) @ pose(link) @ pose(collision)
            vertices = load_stl(path) * scale
            result.append(vertices @ transform[:3, :3].T + transform[:3, 3])
    if not result:
        raise ValueError('no field collision triangles')
    return np.concatenate(result)


def slice_segments(triangle, z, tolerance=SLICE_TOLERANCE_M):
    """Intersect using metre tolerance; coplanar faces emit edges, tangencies emit none."""
    if not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('expected positive finite slice tolerance')
    triangle = np.asarray(sorted(map(tuple, triangle)), dtype=float)
    if triangle.shape != (3, 3) or not np.isfinite(triangle).all() or not np.isfinite(z):
        raise ValueError('expected finite triangle and slice height')
    distances = triangle[:, 2] - z
    distances[np.abs(distances) <= tolerance] = 0
    points = [vertex[:2] for vertex, d in zip(triangle, distances) if d == 0]
    for i, j in combinations(range(3), 2):
        if distances[i] * distances[j] < 0:
            fraction = distances[i] / (distances[i] - distances[j])
            points.append(triangle[i, :2] + fraction * (triangle[j, :2] - triangle[i, :2]))
    distinct = []
    for point in sorted(map(tuple, points)):
        point = np.asarray(point)
        if all(np.linalg.norm(point - other) > tolerance for other in distinct):
            distinct.append(point)
    pairs = list(combinations(distinct, 2))
    if not pairs:
        return []
    if np.all(distances == 0):
        return pairs
    return [max(pairs, key=lambda pair: np.linalg.norm(pair[1] - pair[0]))]


def slice_cells(tris, z, res, origin, shape):
    """Sample intersection segments at res/4; row zero is lowest world y."""
    height, width = shape
    grid = np.zeros(shape, bool)
    for triangle in tris:
        for p, q in slice_segments(triangle, z):
            count = max(2, int(np.linalg.norm(q - p) / (res * 0.25)) + 1)
            xy = p + np.linspace(0, 1, count)[:, None] * (q - p)
            ix = np.floor((xy[:, 0] - origin[0]) / res).astype(int)
            iy = np.floor((xy[:, 1] - origin[1]) / res).astype(int)
            ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
            grid[iy[ok], ix[ok]] = True
    return grid


def rasterize(tris, z, res, origin, shape, grow=1):
    """Keep interior-facing surfaces; unobserved enclosed/exterior cells stay unknown."""
    if res <= 0 or grow < 0 or not np.isfinite([z, res, *origin]).all():
        raise ValueError('invalid raster parameters')
    geom = slice_cells(tris, z, res, origin, shape)
    row, col = np.floor(-np.asarray(origin) / res).astype(int)[::-1]
    if not (0 <= row < shape[0] and 0 <= col < shape[1]) or geom[row, col]:
        raise ValueError('world origin must be a free seed inside the canvas')
    labels, _ = ndimage.label(~geom)
    interior = labels == labels[row, col]
    if interior[0].any() or interior[-1].any() or interior[:, 0].any() or interior[:, -1].any():
        raise ValueError('field interior leaks to canvas boundary')
    seen = geom & ndimage.binary_dilation(interior, iterations=2)
    occupied = ndimage.binary_dilation(seen, iterations=grow) if grow else seen
    image = np.full(shape, UNKNOWN, np.uint8)
    image[interior & ~occupied] = FREE
    image[occupied] = OCCUPIED
    return np.flipud(image)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('yaml', type=Path, help='existing map metadata and image canvas')
    parser.add_argument('world', type=Path, help='sim/world/ARCC_Field_2026.sdf')
    parser.add_argument('out_pgm', type=Path)
    parser.add_argument('--z', type=float, default=0.357296694)
    parser.add_argument('--grow', type=int, default=1)
    args = parser.parse_args()
    meta = yaml.safe_load(args.yaml.read_text())
    validate_metadata(meta)
    with Image.open(args.yaml.parent / meta['image']) as canvas:
        width, height = canvas.size
    image = rasterize(load_world_triangles(args.world), args.z, meta['resolution'],
                      meta['origin'][:2], (height, width), args.grow)
    Image.fromarray(image).save(args.out_pgm)
    values, counts = np.unique(image, return_counts=True)
    print(f'{args.out_pgm}: {width}x{height}, z={args.z}, ' +
          ', '.join(f'{v}={n}' for v, n in zip(values, counts)))


if __name__ == '__main__':
    main()
