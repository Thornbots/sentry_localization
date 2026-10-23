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
        ((-4.067, 0.002, 0), (-4.3, 0.2), 0, 0),
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
        json.dumps(
            dict(
                root=[2, 2, 0],
                chassis_yaw=0,
                team="red",
                raw_odom=[-4.625, 0],
                session_id="test-session",
            )
        )
    )
    return source


def test_room_bundle_without_referee(tmp_path):
    source = make_bundle(tmp_path)
    boot = json.loads((source / "boot.json").read_text())
    boot.update(frame_mode="room", team=None)
    (source / "boot.json").write_text(json.dumps(boot))
    workflow.prepare_room(NS(source=source, output=tmp_path / "room"))
    assert workflow.check(tmp_path / "room")["frame_mode"] == "room"
    boot["frame_mode"] = "field"
    (source / "boot.json").write_text(json.dumps(boot))
    with pytest.raises(ValueError, match="referee team"):
        workflow.validate_bundle(source)


@pytest.mark.parametrize("room,no_referee,team,passes", [
    (True, False, None, True), (False, False, None, False),
    (False, True, "red", True), (False, True, "blue", False),
])
def test_robot_id_zero(monkeypatch, room, no_referee, team, passes):
    import sys

    for name in ("nav_msgs.msg", "dji_serial_bridge.msg", "std_msgs.msg"):
        monkeypatch.setitem(sys.modules, name, NS(Odometry=object, RefSysStatus=object, String=object))
    monkeypatch.setitem(sys.modules, "rclpy.qos", NS(
        qos_profile_sensor_data=object(), QoSProfile=lambda **kw: None,
        DurabilityPolicy=NS(TRANSIENT_LOCAL=1)))

    def subscribe(kind, topic, callback, qos):
        if topic == "/odom":
            callback(NS(header=NS(stamp=NS(sec=1, nanosec=0)),
                        pose=NS(pose=NS(position=NS(x=-4.625, y=0)))))
        elif topic.endswith("ref_sys"):
            callback(NS(robot_id=0, is_on_blue_team=False))
        return object()

    def wait(predicate, timeout):
        if not predicate():
            raise RuntimeError("timeout")

    runtime = NS(subscriptions=[], wait=wait, node=NS(
        create_subscription=subscribe, get_clock=lambda: NS(now=lambda: NS(nanoseconds=10**9))))
    args = NS(room=room, no_referee=no_referee, team=team, timeout=1, max_age=3)
    if passes:
        result = workflow.observe_inputs(runtime, args, require_session=False)
        assert result["team"] == (None if room else team)
    else:
        with pytest.raises((RuntimeError, ValueError)):
            workflow.observe_inputs(runtime, args, require_session=False)


def test_alignment_keeps_graph_boot_frame_and_seeds_root(tmp_path):
    source = make_bundle(tmp_path)
    output = tmp_path / "field"
    workflow.align(NS(source=source, output=output, x=-4.0, y=0.3, yaw=0, team="red"))
    manifest = workflow.check(output)
    assert manifest["initial_root"] == pytest.approx([-4, 0.3, 0])
    assert (source / "map.posegraph").read_bytes() == b"graph-data"
    assert not (output / "field.posegraph").exists()
    assert yaml.safe_load((output / "field.yaml").read_text())["origin"][2] == 0
    with pytest.raises(FileExistsError):
        workflow.align(
            NS(source=source, output=output, x=-4.0, y=0.3, yaw=0, team="red")
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


def test_session_can_be_moved_as_one_directory(tmp_path):
    session = tmp_path / "original"
    session.mkdir()
    source = make_bundle(session)
    workflow.align(
        NS(
            source=source,
            output=session / "deployment",
            x=-4.625,
            y=0,
            yaw=0,
            team=None,
        )
    )
    moved = tmp_path / "moved"
    session.rename(moved)
    assert workflow.check(moved / "deployment")["team"] == "red"


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
        (["mapping", "SESSION", "--room", "real_hardware:=false"], ["use_rf2o:=false"]),
        (
            ["mapping", "SESSION", "--raw-odom", "real_hardware:=false"],
            ["use_rf2o:=false", "seed_from_raw_odom:=true"],
        ),
        (
            ["mapping", "SESSION", "real_hardware:=false"],
            ["use_rf2o:=true", "seed_from_raw_odom:=true", "autosave_map:=true"],
        ),
        (
            [
                "localize",
                "BUNDLE",
                "real_hardware:=false",
            ],
            [
                "use_rf2o:=true",
                "seed_from_raw_odom:=true",
                "amcl_initial_x:=-4.3",
                "amcl_initial_yaw:=0.2",
            ],
        ),
    ],
)
def test_launch_cli_trailing_arguments_and_independent_seeds(
    monkeypatch, tmp_path, command, expected
):
    import sys

    captured = []
    command = [str(tmp_path / "session") if c == "SESSION" else c for c in command]
    monkeypatch.setattr(sys, "argv", ["operator_workflow", *command])
    monkeypatch.setattr(
        workflow,
        "check",
        lambda _: dict(frame_mode="field", initial_root=[-4.3, 0.2, 0.2]),
    )
    monkeypatch.setattr(workflow, "ensure_stack_stopped", lambda: None)
    monkeypatch.setattr(
        workflow, "run_stack", lambda args, session, cmd: captured.extend(cmd)
    )
    workflow.main()
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
        workflow, "run_stack", lambda args, session, cmd: captured.extend(cmd)
    )
    monkeypatch.setattr(
        sys, "argv", ["operator_workflow", "localize", "BUNDLE", "real_hardware:=false"]
    )
    workflow.main()
    assert "use_rf2o:=false" in captured
    assert "amcl_initial_x:=2" in captured
    captured.clear()
    monkeypatch.setattr(
        sys,
        "argv",
        ["operator_workflow", "mapping", "SESSION", "--room", "use_rf2o:=true"],
    )
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 1
    assert not captured


@pytest.mark.parametrize(
    "team,x,yaw,reported",
    [
        ("green", -4.625, 0, 0),
        ("blue", -4.625, 0, 0),
        ("red", 4.625, 0, 0),
        ("red", 0, 0, 0),
        ("red", -4.625, math.pi, 0),
        ("red", -4.625, 180, 0),
        ("red", -4.625, 0, math.pi),
    ],
)
def test_field_rejects_team_sign_heading_and_degrees(team, x, yaw, reported):
    boot = dict(team="red", raw_odom=[-4.625, 0], chassis_yaw=reported)
    with pytest.raises(ValueError):
        workflow.validate_field(boot, team, x, 0, yaw)


@pytest.mark.parametrize("team,sign,heading", [("red", -1, 0), ("blue", 1, math.pi)])
def test_registration_known_wall_in_field_coordinates(team, sign, heading):
    boot = dict(team=team, raw_odom=[sign * 4.625, 0], chassis_yaw=heading)
    workflow.validate_field(boot, team, sign * 4.5, 0.2, heading)
    correction, _ = workflow.registration(
        [sign * 4.625, 0, 0], sign * 4.5, 0.2, heading, heading
    )
    # A wall 1 m toward +x and 2 m toward +y from the parked root.
    assert workflow.transform((sign * 4.625 + 1, 2), correction) == pytest.approx(
        (sign * 4.5 + 1, 2.2)
    )


def test_stale_boot_or_not_returned_refused():
    boot = dict(root=[-4.625, 0, 0], session_id="current")
    workflow.verify_return(boot, [-4.625, 0, 0], "current")
    with pytest.raises(ValueError, match="wrong-session"):
        workflow.verify_return(boot, [-4.625, 0, 0], "old")
    with pytest.raises(ValueError, match="return to boot"):
        workflow.verify_return(boot, [-3, 0, 0], "current")


def test_seed_mismatch_refused():
    boot = dict(team="red", raw_odom=[-4.625, 0])
    workflow.verify_start(boot, dict(boot))
    with pytest.raises(ValueError, match="seed/team mismatch"):
        workflow.verify_start(boot, dict(team="red", raw_odom=[0, 0]))
    with pytest.raises(ValueError, match="seed/team mismatch"):
        workflow.verify_start(boot, dict(team="blue", raw_odom=[-4.625, 0]))


def test_blue_pi_allows_float32_serialization_noise():
    yaw = float(np.float32(math.pi))
    workflow.validate_field(
        dict(team="blue", raw_odom=[4.625, 0], chassis_yaw=yaw), "blue", 4.625, 0, yaw
    )


def test_registration_places_known_wall_pixels():
    meta = metadata()
    meta.update(resolution=0.1, origin=[-5, -1, 0])
    pixels = np.full((20, 30), 254, dtype=np.uint8)
    pixels[:, 20] = 0  # Wall x=-2.95, cell centres from y=-0.95 through 0.95.
    correction, _ = workflow.registration([-4.625, 0, 0], -4.5, 0.2, 0, 0)
    output, raster = workflow.rotate_grid(meta, pixels, correction)
    rows, cols = np.where(raster == 0)
    assert len(cols) == 20
    assert output["origin"][0] + (cols + 0.5) * 0.1 == pytest.approx(
        np.full(20, -2.825)
    )
    ys = output["origin"][1] + (raster.shape[0] - rows - 0.5) * 0.1
    assert sorted(ys) == pytest.approx(np.arange(-0.75, 1.2, 0.1))


def test_negative_scientific_yaw_parses(monkeypatch, tmp_path):
    import sys

    captured = []
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "operator_workflow",
            "align",
            str(tmp_path),
            "--x",
            "-4.625",
            "--y",
            "0",
            "--yaw",
            "-2e-05",
        ],
    )
    monkeypatch.setattr(workflow, "align", lambda args: captured.append(args.yaw))
    workflow.main()
    assert captured == [-2e-05]


@pytest.mark.parametrize("option", ["--odom-x", "--odom-y"])
def test_operator_seed_removed(monkeypatch, option):
    import sys

    monkeypatch.setattr(
        sys, "argv", ["operator_workflow", "mapping", "SESSION", option, "99"]
    )
    with pytest.raises(SystemExit) as result:
        workflow.main()
    assert result.value.code == 2


def test_failed_localize_stops_owned_launch_without_pass(monkeypatch, tmp_path, capsys):
    import subprocess
    import sys

    processes = []
    real_popen = subprocess.Popen

    def launch(*args, **kwargs):
        process = real_popen(*args, **kwargs)
        processes.append(process)
        return process

    def fail(_args):
        raise RuntimeError("away from registered start")

    monkeypatch.setattr(workflow.subprocess, "Popen", launch)
    monkeypatch.setattr(workflow, "convergence", fail)
    with pytest.raises(RuntimeError, match="away from registered start"):
        workflow.run_stack(
            NS(command="localize"),
            tmp_path,
            [sys.executable, "-c", "import time; time.sleep(60)"],
        )
    assert processes[0].poll() is not None
    assert "LOCALIZE PASS" not in capsys.readouterr().out
