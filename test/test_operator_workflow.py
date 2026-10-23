# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.
"""Independent geometry, artifact and convergence failure checks without ROS."""

import importlib.machinery
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
from PIL import Image
import pytest
import yaml

loader = importlib.machinery.SourceFileLoader(
    "operator_workflow", str(Path(__file__).parents[1] / "scripts/operator_workflow")
)
spec = importlib.util.spec_from_loader(loader.name, loader)
workflow = importlib.util.module_from_spec(spec)
loader.exec_module(workflow)


def metadata():
    return dict(
        image="map.pgm",
        origin=[0.0, 0.0, 0.0],
        resolution=1.0,
        negate=0,
        occupied_thresh=0.65,
        free_thresh=0.196,
    )


@pytest.mark.parametrize(
    "boot,measured,heading,reported",
    [
        ((-4.067, 0.002, 0), (-4.3, 0.2), math.pi, math.pi),
        ((2, -1, 0.3), (4, 2), 1.2, -0.4),
        ((0, 0, 0), (0, 0), -math.pi, math.pi),
    ],
)
def test_registration_maps_actual_boot_root(boot, measured, heading, reported):
    correction, root = workflow.registration(boot, *measured, heading, reported)
    assert workflow.transform(boot[:2], correction) == pytest.approx(measured)
    assert workflow.angle(boot[2] + correction[2]) == pytest.approx(root[2])
    assert root[2] == pytest.approx(workflow.angle(heading - reported))


@pytest.mark.parametrize("yaw", [0, math.pi / 2, math.pi, -math.pi / 2])
def test_rotated_raster_preserves_cell_locations(yaw):
    pixels = np.array([[0, 254, 205], [254, 0, 254]], dtype=np.uint8)
    output, raster = workflow.rotate_grid(metadata(), pixels, (4, -3, yaw))
    assert output["origin"][2] == 0
    assert raster.size == pixels.size
    for row in range(2):
        for col in range(3):
            x, y = workflow.transform((col + 0.5, 2 - row - 0.5), (4, -3, yaw))
            dest_col = math.floor(x - output["origin"][0])
            dest_row = raster.shape[0] - 1 - math.floor(y - output["origin"][1])
            assert raster[dest_row, dest_col] == pixels[row, col]


def test_arbitrary_rotation_unknown_padding_and_source_yaw():
    meta = metadata()
    meta["origin"] = [2, -1, 0.2]
    output, raster = workflow.rotate_grid(meta, np.full((20, 30), 254), (3, 4, 0.4))
    assert output["origin"][2] == 0
    assert set(np.unique(raster)) == {205, 254}
    assert np.count_nonzero(raster == 254) == pytest.approx(600, abs=4)


def make_bundle(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    Image.fromarray(np.full((10, 10), 254, dtype=np.uint8)).save(source / "map.pgm")
    (source / "map.yaml").write_text(yaml.safe_dump(metadata()))
    for suffix in ("data", "posegraph"):
        (source / f"map.{suffix}").write_bytes(b"graph-data")
    (source / "boot.json").write_text(
        json.dumps(dict(root=[2, 2, 0], chassis_yaw=math.pi))
    )
    return source


def test_alignment_keeps_graph_boot_frame_and_seeds_root(tmp_path):
    source = make_bundle(tmp_path)
    output = tmp_path / "field"
    workflow.align(
        NS(source=source, output=output, x=-4.0, y=0.3, yaw=math.pi, team="red")
    )
    manifest = workflow.check(output)
    assert manifest["initial_root"] == pytest.approx([-4, 0.3, 0])
    assert (source / "map.posegraph").read_bytes() == b"graph-data"
    assert not (output / "field.posegraph").exists()
    assert yaml.safe_load((output / "field.yaml").read_text())["origin"][2] == 0
    with pytest.raises(FileExistsError):
        workflow.align(
            NS(source=source, output=output, x=-4.0, y=0.3, yaw=math.pi, team="red")
        )


def test_missing_graph_and_boot_are_not_success(tmp_path):
    source = make_bundle(tmp_path)
    (source / "map.data").write_bytes(b"")
    with pytest.raises(ValueError, match="empty"):
        workflow.validate_bundle(source)
    (source / "map.data").write_bytes(b"data")
    (source / "boot.json").unlink()
    with pytest.raises(FileNotFoundError):
        workflow.validate_bundle(source)


def scan(count=360):
    return NS(
        ranges=[1.0] * count,
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2 * math.pi / (count - 1),
    )


def test_variable_lidar_count_and_angles_are_rejected():
    geometry = workflow.scan_geometry(scan())
    assert workflow.geometry_matches(geometry, workflow.scan_geometry(scan()))
    assert not workflow.geometry_matches(geometry, workflow.scan_geometry(scan(341)))
    changed = scan()
    changed.angle_increment *= 1.01
    with pytest.raises(ValueError, match="beam count"):
        workflow.scan_geometry(changed)


def pose(covariance=None, x=1.0, yaw=0.0):
    if covariance is None:
        covariance = np.diag([0.01, 0.01, 0, 0, 0, 0.01]).flatten()
    return NS(
        pose=NS(
            covariance=covariance,
            pose=NS(
                position=NS(x=x, y=2.0),
                orientation=NS(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        )
    )


def accepted(message):
    return workflow.sample_ok(message, [1, 2, 0], 0.25, 0.15, 0.4, 0.25)


def test_convergence_rejects_bad_covariance_and_wrong_hypothesis():
    assert accepted(pose())
    assert not accepted(pose(x=2))
    assert not accepted(pose(yaw=math.pi))
    for value in (float("nan"), float("inf"), -1, 1):
        cov = np.diag([value, 0.01, 0, 0, 0, 0.01]).flatten()
        assert not accepted(pose(cov))
    cov = np.diag([0.01, 0.01, 0, 0, 0, 0.01])
    cov[0, 1] = cov[1, 0] = 0.1
    assert not accepted(pose(cov.flatten()))


def test_room_has_no_field_inputs_and_keeps_boot_coordinates(tmp_path):
    source = make_bundle(tmp_path)
    target = tmp_path / "room"
    workflow.prepare_room(NS(source=source, output=target))
    manifest = workflow.check(target)
    assert manifest["frame_mode"] == "room"
    assert manifest["initial_root"] == [2, 2, 0]
    assert manifest["field_from_boot"] == [0, 0, 0]
    assert "team" not in manifest
    _, original = workflow.read_map(source / "map")
    _, prepared = workflow.read_map(target / "field")
    assert np.array_equal(original, prepared)


@pytest.mark.parametrize(
    "command,expected",
    [
        (["mapping", "--room", "real_hardware:=false"], ["use_rf2o:=false"]),
        (
            ["mapping", "--raw-odom", "--odom-x", "-5", "--odom-y", "0",
             "real_hardware:=false"],
            ["use_rf2o:=false", "initial_x:=-5.0"],
        ),
        (
            ["mapping", "--odom-x", "-4", "--odom-y", "0", "real_hardware:=false"],
            ["use_rf2o:=true", "initial_x:=-4.0"],
        ),
        (
            [
                "localize",
                "BUNDLE",
                "--odom-x",
                "-4",
                "--odom-y",
                "0",
                "real_hardware:=false",
            ],
            [
                "use_rf2o:=true",
                "initial_x:=-4.0",
                "amcl_initial_x:=-4.3",
                "amcl_initial_yaw:=0.2",
            ],
        ),
    ],
)
def test_launch_cli_trailing_arguments_and_independent_seeds(
    monkeypatch, command, expected
):
    import sys

    captured = []
    monkeypatch.setattr(sys, "argv", ["operator_workflow", *command])
    monkeypatch.setattr(
        workflow,
        "check",
        lambda _: dict(frame_mode="field", initial_root=[-4.3, 0.2, 0.2]),
    )
    monkeypatch.setattr(workflow, "ensure_stack_stopped", lambda: None)
    monkeypatch.setattr(
        workflow.subprocess, "call", lambda cmd: captured.extend(cmd) or 0
    )
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 0
    assert "real_hardware:=false" in captured
    for value in expected:
        assert value in captured


def test_room_localize_no_odometry_seed_and_rejects_rf2o_override(monkeypatch):
    import sys

    captured = []
    monkeypatch.setattr(
        workflow, "check", lambda _: dict(frame_mode="room", initial_root=[2, 2, 0])
    )
    monkeypatch.setattr(workflow, "ensure_stack_stopped", lambda: None)
    monkeypatch.setattr(
        workflow.subprocess, "call", lambda cmd: captured.extend(cmd) or 0
    )
    monkeypatch.setattr(
        sys, "argv", ["operator_workflow", "localize", "BUNDLE", "real_hardware:=false"]
    )
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 0
    assert "use_rf2o:=false" in captured
    assert "amcl_initial_x:=2" in captured
    captured.clear()
    monkeypatch.setattr(
        sys, "argv", ["operator_workflow", "mapping", "--room", "use_rf2o:=true"]
    )
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 1
    assert not captured
