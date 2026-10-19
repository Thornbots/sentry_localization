#!/usr/bin/env python3
# Copyright 2026 Thornbots
# Licensed under the Apache License, Version 2.0.
"""Write a saved occupancy map image from the field mesh at the lidar height.

    field_map.py map/clean_map.yaml Field.stl out.pgm --style binary \
        [--center-marker CenterMarker.stl] [--z 0.354] [--grow 1]

The canvas (resolution, origin, size) comes from an existing map yaml, so the
yaml stays valid and only the image changes. The mesh is placed as in the sim
world (sim `ARCC_Field_2026.sdf`: model yaw -pi/2, collision z -0.0133884):
world = (mesh_y, -mesh_x, mesh_z - 0.0133884), the field-centred frame.
Occupied = cells where the plane z cuts the mesh and that border the field
interior, so only surfaces a lidar can see (no outer wall faces), grown by
--grow cells. `--z` is the lidar height: sim robot root z 0.0093 m + 0.345 m.
  binary : 0 occupied, 255 elsewhere (clean_map)
  trinary: 0 occupied, 254 interior free, 205 unknown (ARCC26)
"""
import argparse
import struct
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

MESH_DZ = -0.0133884


def load_world_triangles(path):
    """Binary STL -> (n, 3, 3) vertices in the field-centred world frame."""
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    dtype = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    v = np.frombuffer(data, dtype=dtype, count=count, offset=84)['v'].astype(np.float64)
    return np.stack([v[..., 1], -v[..., 0], v[..., 2] + MESH_DZ], axis=-1)


def slice_cells(tris, z, res, origin, shape):
    """Boolean grid (row 0 = lowest y) of cells crossed by the plane z."""
    height, width = shape
    grid = np.zeros(shape, bool)
    d = tris[..., 2] - z
    for t, dt in zip(tris, d):
        pts = []
        for i in range(3):
            j = (i + 1) % 3
            if dt[i] != dt[j] and ((dt[i] < 0) != (dt[j] < 0) or 0 in (dt[i], dt[j])):
                s = dt[i] / (dt[i] - dt[j])
                pts.append(t[i, :2] + s * (t[j, :2] - t[i, :2]))
        if len(pts) < 2:
            continue
        p, q = pts[0], pts[1]
        n = max(2, int(np.hypot(*(q - p)) / (res * 0.25)) + 1)
        xy = p + np.linspace(0, 1, n)[:, None] * (q - p)
        ix = np.floor((xy[:, 0] - origin[0]) / res).astype(int)
        iy = np.floor((xy[:, 1] - origin[1]) / res).astype(int)
        ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
        grid[iy[ok], ix[ok]] = True
    return grid


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('yaml', help='existing map yaml giving the canvas')
    ap.add_argument('field_stl')
    ap.add_argument('out_pgm')
    ap.add_argument('--style', choices=['binary', 'trinary'], required=True)
    ap.add_argument('--center-marker', help='second mesh in the same frame')
    ap.add_argument('--z', type=float, default=0.354)
    ap.add_argument('--grow', type=int, default=1)
    a = ap.parse_args()

    meta = yaml.safe_load(Path(a.yaml).read_text())
    res, origin = meta['resolution'], meta['origin'][:2]
    width, height = Image.open(Path(a.yaml).parent / meta['image']).size
    tris = load_world_triangles(a.field_stl)
    if a.center_marker:
        tris = np.concatenate([tris, load_world_triangles(a.center_marker)])
    geom = slice_cells(tris, a.z, res, origin, (height, width))
    labels, _ = ndimage.label(~geom)
    interior = labels == labels[int(-origin[1] / res), int(-origin[0] / res)]
    seen = geom & ndimage.binary_dilation(interior, iterations=2)
    occ = ndimage.binary_dilation(seen, iterations=a.grow) if a.grow else seen
    if a.style == 'binary':
        img = np.where(occ, 0, 255).astype(np.uint8)
    else:
        img = np.full((height, width), 205, np.uint8)
        img[interior & ~occ] = 254
        img[occ] = 0
    Image.fromarray(np.flipud(img)).save(a.out_pgm)
    print(f'{a.out_pgm}: {width}x{height}, {int(occ.sum())} occupied cells')


if __name__ == '__main__':
    main()
