# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed under the License
# is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and limitations under the License.
"""Exercise the seed gate and EKF handoff without a ROS installation."""

import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest


loader = importlib.machinery.SourceFileLoader(
    'seeded_ekf', str(Path(__file__).parents[1] / 'scripts/seeded_ekf'))
spec = importlib.util.spec_from_loader(loader.name, loader)
gate = importlib.util.module_from_spec(spec)
loader.exec_module(gate)


STARTS = ((-2.7, 0.6), (4.1, -1.2))
START_ARGS = ['--red-start-x', '-2.7', '--red-start-y', '0.6',
              '--blue-start-x', '4.1', '--blue-start-y', '-1.2']


@pytest.fixture(autouse=True)
def fake_ament(monkeypatch):
    monkeypatch.setitem(sys.modules, 'ament_index_python', NS())
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', NS(
        get_package_prefix=lambda name: '/overlay/' + name))


def odom(seconds=10, nanos=0, x=-4.067, y=0.002):
    return NS(header=NS(stamp=NS(sec=seconds, nanosec=nanos)),
              pose=NS(pose=NS(position=NS(x=x, y=y))))


@pytest.mark.parametrize('message,now', [
    (odom(seconds=0), 0),
    (odom(), 10_500_000_001),
    (odom(seconds=11), 10_000_000_000),
    (odom(x=float('nan')), 10_000_000_000),
    (odom(y=float('inf')), 10_000_000_000),
])
def test_reject_unusable_seed(message, now):
    assert gate.initial_state(message, now) is None


def test_seed_retains_actual_raw_xy_and_heading_fixed_state():
    assert gate.initial_state(odom(), 10_500_000_000) == [-4.067, 0.002] + [0.0] * 13


def fake_ros(monkeypatch, messages):
    """Model a spinning ROS clock and queued raw odometry callbacks."""
    node = NS(destroyed=False, destroyed_subscriptions=[], callbacks={},
              now_ns=10_000_000_000, errors=[])
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=node.now_ns))
    node.get_logger = lambda: NS(info=lambda text: None, error=node.errors.append)

    def subscribe(message_type, topic, callback, qos):
        node.callbacks[topic] = callback
        return topic

    node.create_subscription = subscribe
    node.destroy_subscription = node.destroyed_subscriptions.append
    node.destroy_node = lambda: setattr(node, 'destroyed', True)

    def create_node(name, parameter_overrides):
        node.sim_time = parameter_overrides[0].value
        return node

    def spin_once(node, timeout_sec):
        assert 0 < timeout_sec <= 0.1
        if messages:
            event = messages.pop(0)
            if isinstance(event, tuple):
                topic, message, node.now_ns = event
            else:
                topic, message = '/odom', event
            node.callbacks[topic](message)

    monkeypatch.setitem(sys.modules, 'rclpy', NS(
        init=lambda args: None, create_node=create_node, ok=lambda: True,
        spin_once=spin_once, try_shutdown=lambda: None))
    monkeypatch.setitem(sys.modules, 'rclpy.parameter', NS(
        Parameter=lambda name, value: NS(value=value)))
    monkeypatch.setitem(sys.modules, 'rclpy.qos', NS(qos_profile_sensor_data=object()))
    monkeypatch.setitem(sys.modules, 'nav_msgs', NS())
    monkeypatch.setitem(sys.modules, 'nav_msgs.msg', NS(Odometry=object()))
    monkeypatch.setitem(sys.modules, 'dji_serial_bridge', NS())
    monkeypatch.setitem(sys.modules, 'dji_serial_bridge.msg', NS(RefSysStatus=object()))
    return node


def test_wait_skips_bad_messages_then_seeds_and_cleans_up(monkeypatch):
    messages = [odom(seconds=0), odom(seconds=1), odom(x=float('nan')), odom()]
    node = fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 100.0)
    assert gate.wait_for_seed(True, starts=STARTS)[:2] == [-4.067, 0.002]
    assert messages == []
    assert node.sim_time is True
    assert node.destroyed and node.destroyed_subscriptions == ['/odom']


@pytest.mark.parametrize('blue', [False, True])
def test_team_gate_adds_start_to_raw_displacement_after_assignment(monkeypatch, blue):
    start, start_y = STARTS[blue]
    messages = [
        odom(x=0),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=0, is_on_blue_team=not blue), 10_000_000_000),
        odom(x=0),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=blue), 10_100_000_000),
        ('/odom', odom(x=0), 10_100_000_000),  # Captured before assignment.
        ('/odom', odom(nanos=200_000_000, x=start), 10_200_000_000),
        ('/odom', odom(nanos=200_000_000, x=0, y=0.301), 10_200_000_000),
        ('/odom', odom(nanos=200_000_000, x=0.2, y=-0.1), 10_200_000_000),
    ]
    node = fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    assert gate.wait_for_seed(False, starts=STARTS, require_team=True) == [start + 0.2, start_y - 0.1] + [0.0] * 13
    assert messages == []
    assert node.destroyed_subscriptions == ['/dji_serial_bridge/ref_sys', '/odom']
    assert node.destroyed


@pytest.mark.parametrize('backend', ['amcl', 'slam'])
@pytest.mark.parametrize('blue', [False, True])
def test_backend_exec_waits_for_first_valid_team_then_sets_only_team_prior(
        monkeypatch, backend, blue):
    messages = [
        odom(x=0),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=0, is_on_blue_team=not blue), 10_000_000_000),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=blue), 10_000_000_000),
    ]
    node = fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    executed = []

    def execute(path, command):
        assert messages == []  # AMCL cannot exist before valid REF_SYS is consumed.
        assert node.destroyed
        executed.append((path, command))

    monkeypatch.setattr(gate.os, 'execvp', execute)
    forwarded = ['--ros-args', '--params-file', '/tmp/backend.yaml']
    assert gate.main(START_ARGS + ['--seed-use-sim-time', 'false', '--backend', backend, *forwarded]) == 0
    path, command = executed[0]
    assert path == '/overlay/sentry_localization/lib/sentry_localization/localization_backend'
    assert command[1:5] == [backend, *forwarded]
    start, start_y = STARTS[blue]
    if backend == 'amcl':
        assert command[5:] == ['-p', 'set_initial_pose:=true', '-p', f'initial_pose.x:={start}',
                               '-p', f'initial_pose.y:={start_y}', '-p', 'initial_pose.yaw:=0.0']
    else:
        assert command[5:] == ['-p', 'map_start_pose:=' + json.dumps([start, start_y, 0.0])]


@pytest.mark.parametrize('backend', [None, 'amcl', 'slam'])
def test_no_referee_never_execs_even_after_old_timeout(monkeypatch, backend, capsys):
    messages = [odom(x=0),
                ('/dji_serial_bridge/ref_sys', NS(robot_id=0, is_on_blue_team=True), 10_000_000_000)]
    node = fake_ros(monkeypatch, messages)
    monkeypatch.setattr(sys.modules['rclpy'], 'ok', lambda: bool(messages))
    wall_times = iter([0.0, 30.0, 30.0, 60.0, 60.0])
    monkeypatch.setattr(gate.time, 'monotonic', lambda: next(wall_times))
    executed = []
    monkeypatch.setattr(gate.os, 'execvp', lambda *args: executed.append(args))
    args = ['--seed-use-sim-time', 'false', '--require-team']
    if backend:
        args += ['--backend', backend]
    assert gate.main(START_ARGS + args) == 1
    assert executed == []
    assert node.destroyed
    assert 'ROS shut down' in capsys.readouterr().err


def test_wall_timeout_with_stopped_ros_clock_never_starts_ekf(monkeypatch, capsys):
    node = fake_ros(monkeypatch, [])
    wall_times = iter([0.0, 0.0, 15.0])
    monkeypatch.setattr(gate.time, 'monotonic', lambda: next(wall_times))
    executed = []
    monkeypatch.setattr(gate.os, 'execvp', lambda *args: executed.append(args))
    assert gate.main(START_ARGS + ['--seed-use-sim-time', 'true']) == 1
    assert executed == []
    assert 'EKF was not started' in capsys.readouterr().err
    assert node.destroyed and node.destroyed_subscriptions == ['/odom']
    assert 'EKF was not started' in node.errors[0]


def test_explicit_practice_team_adds_start_to_raw_seed(monkeypatch):
    messages = [odom(x=0.25)]
    fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    assert gate.wait_for_seed(False, starts=STARTS, explicit_team='blue')[:2] == pytest.approx([4.35, -1.198])
    assert messages == []


def test_exec_preserves_launch_configuration_and_overrides_only_seed(monkeypatch):
    seed = [-4.067, 0.002] + [0.0] * 13
    received = []

    def wait(sim, require_team, explicit_team, *, starts):
        assert starts == STARTS
        received.append((sim, require_team))
        return seed

    monkeypatch.setattr(gate, 'wait_for_seed', wait)
    executed = []
    monkeypatch.setattr(gate.os, 'execvp', lambda *args: executed.append(args))
    forwarded = ['--ros-args', '--params-file', '/tmp/ekf.yaml',
                 '-r', '__node:=ekf_filter_node', '-r',
                 'odometry/filtered:=/localization/odom']
    assert gate.main(START_ARGS + ['--seed-use-sim-time', 'false', '--require-team', *forwarded]) == 0
    assert received == [(False, True)]
    executable, command = executed[0]
    assert executable == '/overlay/robot_localization/lib/robot_localization/ekf_node'
    assert command[0] == executable
    assert command[1:-2] == forwarded
    assert command[-2] == '-p'
    assert json.loads(command[-1].removeprefix('initial_state:=')) == seed


def load_launch(monkeypatch, path):
    """Capture launch actions while exercising the real launch factory."""
    class Action:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    class Opaque(Action):
        pass

    modules = {
        'ament_index_python': NS(),
        'ament_index_python.packages': NS(get_package_share_directory=lambda name: '/share/' + name),
        'launch': NS(LaunchDescription=lambda actions: actions),
        'launch.actions': NS(DeclareLaunchArgument=Action, OpaqueFunction=Opaque,
                             IncludeLaunchDescription=Action, EmitEvent=Action,
                             RegisterEventHandler=Action, GroupAction=Action),
        'launch.conditions': NS(IfCondition=Action, UnlessCondition=Action),
        'launch.event_handlers': NS(OnProcessExit=Action),
        'launch.events': NS(Shutdown=Action),
        'launch.launch_description_sources': NS(PythonLaunchDescriptionSource=Action),
        'launch.substitutions': NS(LaunchConfiguration=Action, PythonExpression=Action,
                                  Command=Action, EnvironmentVariable=Action),
        'launch_ros': NS(),
        'launch_ros.actions': NS(Node=Action),
        'launch_ros.parameter_descriptions': NS(ParameterValue=Action),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location('test_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.generate_launch_description()
    groups = [a for a in actions if 'actions' in a.kwargs]
    for group in groups:
        actions.extend(group.kwargs['actions'])
    return actions, Opaque


@pytest.mark.parametrize('require_team', [False, True])
def test_launch_gates_amcl_independently_of_room_navigation(monkeypatch, require_team):
    actions, opaque = load_launch(
        monkeypatch, Path(__file__).parents[1] / 'launch/localization.launch.py')
    factory = next(action.kwargs['function'] for action in actions if isinstance(action, opaque))
    shutdown, node = factory(NS(launch_configurations={
        'seed_from_raw_odom': 'false', 'seed_require_team': str(require_team).lower(),
        'initial_x': 'invalid', 'initial_y': 'invalid', 'seed_explicit_team': 'blue'}))
    handler, = shutdown.args
    assert handler.kwargs['target_action'] is node
    assert node.kwargs['executable'] == 'seeded_ekf'
    assert ('--require-team' in node.kwargs['arguments']) == require_team
    if not require_team:
        assert node.kwargs['arguments'][-2:] == ['--team', 'blue']
    assert 'initial_state' not in node.kwargs['parameters'][1]
    amcl = next(a for a in actions if a.kwargs.get('name') == 'localization_container')
    assert amcl.kwargs['executable'] == 'seeded_ekf'
    assert amcl.kwargs['arguments'][-2:] == ['--backend', 'amcl']
    assert not any(key.startswith('initial_pose') for key in amcl.kwargs['parameters'][1])
    scan = next(a for a in actions if a.kwargs.get('name') == 'scan_navigation_publisher')
    assert scan.kwargs['remappings'] == [('/odom', '/scan_odom'),
                                        ('/localization/odom', '/localization/scan_odom')]


def test_launch_only_saved_localization_gets_team_map_prior_mapping_keeps_zero(monkeypatch):
    actions, _ = load_launch(
        monkeypatch, Path(__file__).parents[1] / 'launch/localization.launch.py')
    slam = [a for a in actions if a.kwargs.get('name') == 'slam_container']
    assert len(slam) == 3
    localization, loaded_mapping, blank_mapping = slam
    assert localization.kwargs['executable'] == 'seeded_ekf'
    assert localization.kwargs['arguments'][-2:] == ['--backend', 'slam']
    for mapping in (loaded_mapping, blank_mapping):
        assert mapping.kwargs['executable'] == 'localization_backend'
        assert mapping.kwargs['arguments'] == ['slam']
    assert loaded_mapping.kwargs['parameters'][1]['map_start_pose'] == [0.0, 0.0, 0.0]
    assert 'map_start_pose' not in blank_mapping.kwargs['parameters'][1]
    assert blank_mapping.kwargs['parameters'][1]['zero_origin_mapping'] is True
    assert loaded_mapping.kwargs['parameters'][1]['zero_origin_mapping'] is False


def test_room_navigation_without_explicit_team_retains_raw_zero_origin(monkeypatch):
    messages = [odom(x=0.15, y=-0.1)]
    fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    assert gate.wait_for_seed(False, starts=STARTS, require_team=False) == [0.15, -0.1] + [0.0] * 13


def test_required_referee_ignores_conflicting_explicit_practice_team(monkeypatch):
    messages = [
        odom(x=0),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=False), 10_000_000_000),
        ('/odom', odom(nanos=100_000_000, x=0), 10_100_000_000),
    ]
    fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    assert gate.wait_for_seed(False, starts=STARTS, require_team=True, explicit_team='blue')[0] == -2.7


def test_valid_referee_without_fresh_raw_seed_times_out_without_ekf(monkeypatch, capsys):
    messages = [('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=True),
                 10_000_000_000), odom(seconds=1)]
    node = fake_ros(monkeypatch, messages)
    wall_times = iter([0.0, 0.0, 0.0, 0.0, 15.0])
    monkeypatch.setattr(gate.time, 'monotonic', lambda: next(wall_times))
    executed = []
    monkeypatch.setattr(gate.os, 'execvp', lambda *args: executed.append(args))
    assert gate.main(START_ARGS + ['--seed-use-sim-time', 'false', '--require-team']) == 1
    assert executed == []
    assert node.destroyed
    assert 'within 15 s' in capsys.readouterr().err
