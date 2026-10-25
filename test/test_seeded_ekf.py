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
    assert gate.wait_for_seed(True)[:2] == [-4.067, 0.002]
    assert messages == []
    assert node.sim_time is True
    assert node.destroyed and node.destroyed_subscriptions == ['/odom']


@pytest.mark.parametrize('blue', [False, True])
@pytest.mark.parametrize('reanchor_first', [False, True])
def test_team_gate_waits_for_assignment_then_new_matching_odom(monkeypatch, blue, reanchor_first):
    start = 4.625 if blue else -4.625
    assigned_ns = 10_100_000_000
    messages = [
        odom(x=-start),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=0, is_on_blue_team=blue), assigned_ns),
        ('/odom', odom(nanos=100_000_000, x=-start), assigned_ns),
    ]
    if reanchor_first:
        messages.append(('/odom', odom(nanos=100_000_000, x=start), assigned_ns))
    messages.extend([
        ('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=blue), assigned_ns),
        ('/odom', odom(x=start), assigned_ns),  # Captured before assignment, delivered after it.
        ('/odom', odom(nanos=100_000_000, x=start), assigned_ns),
        ('/odom', odom(nanos=200_000_000, x=-start), 10_200_000_000),
        ('/odom', odom(nanos=200_000_000, x=start, y=0.301), 10_200_000_000),
        ('/odom', odom(nanos=200_000_000, x=start + 0.2, y=0.2), 10_200_000_000),
    ])
    node = fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    assert gate.wait_for_seed(False, require_team=True)[:2] == [start + 0.2, 0.2]
    assert messages == []
    assert node.destroyed_subscriptions == ['/dji_serial_bridge/ref_sys', '/odom']
    assert node.destroyed


def test_team_change_requires_new_odom_from_new_start(monkeypatch):
    messages = [
        ('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=False), 10_000_000_000),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=True), 10_100_000_000),
        ('/odom', odom(nanos=150_000_000, x=-4.625), 10_150_000_000),
        ('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=True), 10_200_000_000),
        ('/odom', odom(nanos=150_000_000, x=4.625), 10_200_000_000),
    ]
    fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    assert gate.wait_for_seed(False, require_team=True)[:2] == [4.625, 0.002]
    assert messages == []


@pytest.mark.parametrize('messages', [
    [odom(x=-4.625)],
    [('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=True), 10_000_000_000)],
    [('/dji_serial_bridge/ref_sys', NS(robot_id=7, is_on_blue_team=True), 10_000_000_000),
     odom(x=-4.625)],
])
def test_team_or_matching_odom_timeout_never_execs(monkeypatch, messages, capsys):
    node = fake_ros(monkeypatch, list(messages))
    wall_times = iter([0.0] * (len(messages) + 1) + [15.0])
    monkeypatch.setattr(gate.time, 'monotonic', lambda: next(wall_times))
    executed = []
    monkeypatch.setattr(gate.os, 'execvp', lambda *args: executed.append(args))
    assert gate.main(['--seed-use-sim-time', 'false', '--require-team']) == 1
    assert executed == []
    assert 'referee team and subsequent fresh raw /odom' in capsys.readouterr().err
    assert node.errors and node.destroyed
    assert node.destroyed_subscriptions == ['/dji_serial_bridge/ref_sys', '/odom']


def test_wall_timeout_with_stopped_ros_clock_never_starts_ekf(monkeypatch, capsys):
    node = fake_ros(monkeypatch, [])
    wall_times = iter([0.0, 0.0, 15.0])
    monkeypatch.setattr(gate.time, 'monotonic', lambda: next(wall_times))
    executed = []
    monkeypatch.setattr(gate.os, 'execvp', lambda *args: executed.append(args))
    assert gate.main(['--seed-use-sim-time', 'true']) == 1
    assert executed == []
    assert 'EKF was not started' in capsys.readouterr().err
    assert node.destroyed and node.destroyed_subscriptions == ['/odom']
    assert 'EKF was not started' in node.errors[0]


def test_explicit_practice_team_rechecks_seed_after_restart(monkeypatch):
    messages = [odom(x=-4.625), odom(x=0), odom(x=4.625)]
    fake_ros(monkeypatch, messages)
    monkeypatch.setattr(gate.time, 'monotonic', lambda: 0.0)
    assert gate.wait_for_seed(False, explicit_team='blue')[:2] == [4.625, 0.002]
    assert messages == []


def test_exec_preserves_launch_configuration_and_overrides_only_seed(monkeypatch):
    seed = [-4.067, 0.002] + [0.0] * 13
    received = []

    def wait(sim, require_team, explicit_team):
        received.append((sim, require_team))
        return seed

    monkeypatch.setattr(gate, 'wait_for_seed', wait)
    executed = []
    monkeypatch.setattr(gate.os, 'execvp', lambda *args: executed.append(args))
    forwarded = ['--ros-args', '--params-file', '/tmp/ekf.yaml',
                 '-r', '__node:=ekf_filter_node', '-r',
                 'odometry/filtered:=/localization/odom']
    assert gate.main(['--seed-use-sim-time', 'false', '--require-team', *forwarded]) == 0
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


@pytest.mark.parametrize('seeded', [False, True])
@pytest.mark.parametrize('require_team', [False, True])
def test_launch_selects_gate_without_evaluating_arbitrary_initial_xy(
        monkeypatch, seeded, require_team):
    actions, opaque = load_launch(
        monkeypatch, Path(__file__).parents[1] / 'launch/localization.launch.py')
    declaration = next(action for action in actions if action.args == ('seed_from_raw_odom',))
    assert declaration.kwargs['default_value'] == 'false'
    factory = next(action.kwargs['function'] for action in actions if isinstance(action, opaque))
    team_declaration = next(action for action in actions if action.args == ('seed_require_team',))
    assert team_declaration.kwargs['default_value'] == 'false'
    config = {'seed_from_raw_odom': str(seeded).lower(),
              'seed_require_team': str(require_team).lower()}
    if not seeded:
        config.update(initial_x='1.25', initial_y='-2.5')
    shutdown, node = factory(NS(launch_configurations=config))
    handler, = shutdown.args
    assert handler.kwargs['target_action'] is node
    event = handler.kwargs['on_exit'][0].kwargs['event']
    assert event.kwargs['reason'] == 'EKF process exited'
    assert node.kwargs['executable'] == ('seeded_ekf' if seeded else 'ekf_node')
    assert node.kwargs['parameters'][0] == '/share/sentry_localization/config/ekf.yaml'
    params = node.kwargs['parameters'][1]
    if seeded:
        assert 'initial_state' not in params
        assert node.kwargs['arguments'][0] == '--seed-use-sim-time'
        assert ('--require-team' in node.kwargs['arguments']) == require_team
    else:
        assert params['initial_state'] == [1.25, -2.5] + [0.0] * 13
    assert node.kwargs['remappings'] == [('odometry/filtered', '/localization/odom')]


def test_auto_forwards_seed_and_map_save_directory(monkeypatch):
    path = Path(__file__).parents[2] / 'thornbots_pkg/launch/auto.launch.py'
    if not path.is_file():
        pytest.skip('Workspace auto-launch forwarding check requires sibling thornbots_pkg')
    actions, _ = load_launch(monkeypatch, path)
    includes = [action for action in actions if 'launch_arguments' in action.kwargs]
    assert len(includes) == 1
    arguments = dict(includes[0].kwargs['launch_arguments'])
    for name in ('seed_from_raw_odom', 'seed_require_team', 'seed_explicit_team', 'map_save_dir'):
        assert arguments[name].args == (name,)
    declaration = next(action for action in actions if action.args == ('seed_from_raw_odom',))
    assert declaration.kwargs['default_value'] == 'false'


def test_raw_bootstrap_excludes_command_nodes(monkeypatch):
    path = Path(__file__).parents[2] / 'thornbots_pkg/launch/auto.launch.py'
    if not path.is_file():
        pytest.skip('Workspace bootstrap check requires sibling thornbots_pkg')
    actions, _ = load_launch(monkeypatch, path)
    group = next(a for a in actions if 'actions' in a.kwargs)
    guarded = group.kwargs['actions']
    names = {a.kwargs.get('name') for a in guarded}
    assert {'mcb_relay', 'point_to_cv_target', 'target_selector', 'target_tracker'} <= names
    outside = [a.kwargs.get('name') for a in actions if a not in guarded]
    assert {name for name in outside if name} == {'dji_serial_bridge', 'pose_translator'}
    declaration = next(action for action in actions if action.args == ('seed_require_team',))
    assert declaration.kwargs['default_value'] == 'false'
