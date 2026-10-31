# Copyright 2026 Thornbots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch map->nav_odom backends and the navigation pose estimator.

use_rf2o selects EKF fusion or raw-pose passthrough in aligned navigation axes.
Raw odom->root and translation-only nav_odom->odom TF belong to thornbots_pkg.
The EKF input adapter pins root heading without reading its output TF.
See README.md for the backend table, frame contract and map loading.
"""
import os
import signal

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import SignalProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('sentry_localization')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rf2o_params_file = os.path.join(pkg_share, 'config', 'rf2o.yaml')
    amcl_params_file = os.path.join(pkg_share, 'config', 'amcl.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description="Forwarded from thornbots_pkg/auto.launch.py's "
        'real_hardware-derived value.'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='nav_odom',
        description='Frame slam_toolbox/amcl/ekf_node treat as their '
        'navigation reference, with axes aligned to raw odom.'
    )

    load_map_arg = DeclareLaunchArgument(
        'load_map', default_value='true',
        description="Deserialize map_file's saved pose graph at startup and "
        'continue from it instead of starting blank. Only '
        'affects localization_mode:=slam/mapping (and only '
        'actually works for those modes against a map_file '
        'that has a .posegraph/.data, see map_file '
        'below -- none ships); amcl always loads '
        "map_file's .yaml regardless, and localization_mode:=none "
        'runs no map node at all.'
    )
    map_file_arg = DeclareLaunchArgument(
        'map_file', default_value=os.path.join(pkg_share, 'map', 'clean_map'),
        description='Path (no extension) to the map to use: slam_toolbox '
        'reads <map_file>.posegraph/.data (see '
        'slam_toolbox/srv/SerializePoseGraph), amcl reads '
        '<map_file>.yaml (see nav2_map_server). Same basename, '
        'both refer to the same saved map. Default is '
        'clean_map, in the field frame -- it only has a .yaml/.pgm '
        '(map_server-ready, so localization_mode:=amcl works '
        'against it out of the box). No map here has a '
        '.posegraph/.data, so localization_mode:=slam/mapping with '
        'load_map:=true fails to deserialize until you pass the '
        'map_file of one a mapping run saved (autosave_map).'
    )

    autosave_map_arg = DeclareLaunchArgument(
        'autosave_map', default_value='false',
        description='mapping only: map_autosaver saves the map every '
        'map_save_period_s into map_save_dir/<boot time>/.'
    )
    map_save_dir_arg = DeclareLaunchArgument(
        'map_save_dir', default_value='/workspaces/isaac_ros-dev/maps',
        description='Where map_autosaver writes; the bind-mounted workspace '
        'outlives the container.'
    )
    map_save_period_s_arg = DeclareLaunchArgument(
        'map_save_period_s', default_value='30.0',
        description='Seconds between map_autosaver saves.'
    )

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode', default_value='amcl',
        choices=['slam', 'mapping', 'amcl', 'none'],
        description='Selects who owns map->nav_odom -- see the README '
        'localization_mode table for what each of slam/mapping/amcl/none '
        'actually launches. Independent of use_rf2o, which '
        'estimates the nav_odom->root navigation pose.'
    )
    use_rf2o_arg = DeclareLaunchArgument(
        'use_rf2o', default_value='true',
        description='Whether rf2o informs the nav_odom->root navigation pose '
        '(rf2o_laser_odometry_node, fused by ekf_node) instead of '
        'passing /odom through raw. Independent of localization_mode -- '
        'layers on top of slam/mapping/amcl/none.'
    )
    localization_mode = LaunchConfiguration('localization_mode')
    ekf_selected = PythonExpression(
        ["'", LaunchConfiguration('use_rf2o'), "' == 'true'"]
    )
    amcl_selected = PythonExpression(
        ["'", localization_mode, "' == 'amcl'"]
    )
    mapping_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping'"]
    )
    passthrough_selected = PythonExpression(
        ["'", LaunchConfiguration('use_rf2o'), "' != 'true'"]
    )
    slam_toolbox_with_map_selected = PythonExpression(
        ["'", localization_mode, "' in ('slam', 'mapping') and '",
         LaunchConfiguration('load_map'), "' == 'true'"]
    )
    slam_toolbox_no_map_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping' and '",
         LaunchConfiguration('load_map'), "' == 'false'"]
    )
    backend_pose_topic = PythonExpression(
        ["'/amcl_pose' if '", localization_mode, "' == 'amcl' else '/pose'"]
    )
    slam_toolbox_mode_param = PythonExpression(
        ["'mapping' if '", localization_mode, "' == 'mapping' "
         "else 'localization'"]
    )
    # Map saving/updating (slam_toolbox's use_map_saver, overriding
    # config/slam.yaml's baked-in value) is only ever enabled in mapping
    # mode -- never a side effect of ordinary localization/amcl/ekf
    # running.
    map_yaml_file = PythonExpression(
        ["'", LaunchConfiguration('map_file'), "' + '.yaml'"]
    )

    # UDP-only FastDDS profile (config/fastdds_no_shm.xml): SharedMemTransport
    # init was seen hanging at node startup when /dev/shm held stale
    # fastrtps_* segments from SIGKILLed runs. Same fix as thornbots_pkg's
    # pose_translator/odom_tf_broadcaster.
    passthrough_odom_node = Node(
        package='sentry_localization',
        executable='passthrough_odom_publisher',
        name='passthrough_odom_publisher',
        output='screen',
        condition=IfCondition(passthrough_selected),
        parameters=[{'use_sim_time': use_sim_time,
                     'navigation_frame': LaunchConfiguration('odom_frame')}],
        additional_env={
            'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
                pkg_share, 'config', 'fastdds_no_shm.xml'
            )
        },
    )

    ekf_input_node = Node(
        package='sentry_localization',
        executable='ekf_input_publisher',
        name='ekf_input_publisher',
        output='screen',
        condition=IfCondition(ekf_selected),
        parameters=[{'use_sim_time': use_sim_time,
                     'navigation_frame': LaunchConfiguration('odom_frame')}],
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
            pkg_share, 'config', 'fastdds_no_shm.xml')},
    )

    def make_ekf(context):
        seeded = context.launch_configurations['seed_from_raw_odom'].lower() == 'true'
        initial = {} if seeded else {'initial_state': [
            float(context.launch_configurations['initial_x']),
            float(context.launch_configurations['initial_y'])] + [0.0] * 13}
        arguments = ['--seed-use-sim-time', use_sim_time] if seeded else []
        if seeded and context.launch_configurations['seed_require_team'].lower() == 'true':
            arguments.append('--require-team')
        if seeded and context.launch_configurations.get('seed_explicit_team', ''):
            arguments.extend(['--team', context.launch_configurations['seed_explicit_team']])
        node = Node(
            package='sentry_localization' if seeded else 'robot_localization',
            executable='seeded_ekf' if seeded else 'ekf_node',
            arguments=arguments,
            name='ekf_filter_node',
            output='screen',
            condition=IfCondition(ekf_selected),
            remappings=[('odometry/filtered', '/localization/odom')],
            parameters=[
                ekf_params_file,
                {
                    'use_sim_time': use_sim_time,
                    'odom_frame': LaunchConfiguration('odom_frame'),
                    'base_link_frame': 'root',
                    # Must match odom_frame, not base_link_frame -- see
                    # config/ekf.yaml's comment on world_frame.
                    'world_frame': LaunchConfiguration('odom_frame'),
                    'publish_tf': False,
                    **initial,
                },
            ],
        )

        shutdown = RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='EKF process exited'))]))
        return [shutdown, node]

    ekf_node = OpaqueFunction(function=make_ekf)

    # Only used when use_rf2o:=true; nothing else reads /scan_odom.
    scan_odom_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        condition=IfCondition(ekf_selected),
        # Covariance and match confidence; see config/rf2o.yaml.
        parameters=[rf2o_params_file, {
            'laser_scan_topic': '/scan',
            'odom_topic': '/scan_odom',
            'publish_tf': False,
            'base_frame_id': 'root',
            'odom_frame_id': LaunchConfiguration('odom_frame'),
            'init_pose_from_topic': '/odom',
            # root is heading-fixed (ekf.yaml); without this rf2o's
            # heading drifts and rotates its x/y.
            'fixed_heading': True,
            # Seed each scan match with wheel odometry's motion; rf2o's own
            # constant-velocity guess undershoots every move from rest.
            'odom_prior_topic': '/odom',
            'use_sim_time': use_sim_time,
        }],
    )

    # map->root for mcb_relay; covariance adds the backend's own pose
    # covariance (amcl's /amcl_pose, slam_toolbox's /pose).
    # `fix` freezes substitutions to text for nodes launched from event handlers:
    # the include's launch-configuration scope is gone by the time those run.
    def map_pose(topic, fix=lambda sub: sub):
        return Node(
            package='sentry_localization', executable='map_pose_publisher',
            name='map_pose_publisher', output='screen',
            parameters=[{'use_sim_time': fix(use_sim_time), 'backend_pose_topic': topic}])

    def map_saver(fix=lambda sub: sub):
        return Node(
            package='sentry_localization', executable='map_autosaver',
            name='map_autosaver', output='screen',
            parameters=[{
                'use_sim_time': fix(use_sim_time),
                'periodic_enabled': ParameterValue(
                    fix(LaunchConfiguration('autosave_map')), value_type=bool),
                'save_dir': fix(LaunchConfiguration('map_save_dir')),
                'period_s': ParameterValue(
                    fix(LaunchConfiguration('map_save_period_s')), value_type=float)}])

    # Construct backend components locally; no load_node RPC can race discovery.
    amcl_container = Node(
        package='sentry_localization', executable='localization_backend',
        name='localization_container', arguments=['amcl'], output='screen',
        condition=IfCondition(amcl_selected),
        parameters=[amcl_params_file, {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
            'odom_frame_id': LaunchConfiguration('odom_frame'),
            'base_frame_id': 'root',
            'initial_pose.x': ParameterValue(
                LaunchConfiguration('amcl_initial_x'), value_type=float),
            'initial_pose.y': ParameterValue(
                LaunchConfiguration('amcl_initial_y'), value_type=float),
            'initial_pose.yaw': ParameterValue(
                LaunchConfiguration('amcl_initial_yaw'), value_type=float),
            'global_frame_id': 'map',
            'scan_topic': '/scan',
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }])

    def slam_container(condition, load_map, mapping=False, fix=lambda sub: sub):
        params = {
            'use_sim_time': fix(use_sim_time),
            'odom_frame': fix(LaunchConfiguration('odom_frame')),
            'mode': 'mapping' if mapping else slam_toolbox_mode_param,
            'use_map_saver': True if mapping else ParameterValue(
                mapping_selected, value_type=bool),
            'use_lifecycle_manager': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }
        if load_map:
            params.update({
                'map_file_name': LaunchConfiguration('map_file'),
                'map_start_pose': [0.0, 0.0, 0.0],
            })
        return Node(
            package='sentry_localization', executable='localization_backend',
            name='slam_container', arguments=['mapping' if mapping else PythonExpression(
                ["'mapping' if '", localization_mode, "' == 'mapping' else 'slam'"])],
            output='screen',
            condition=None if condition is None else IfCondition(condition),
            parameters=[slam_params_file, params])

    slam_with_map_container = slam_container(slam_toolbox_with_map_selected, True)
    slam_no_map_container = slam_container(slam_toolbox_no_map_selected, False)

    def managed_backend(context):
        mode = localization_mode.perform(context)
        map_pose_node = map_pose(backend_pose_topic)
        if mode == 'mapping':
            return [map_pose_node, map_saver(),
                    slam_with_map_container, slam_no_map_container]
        if mode == 'amcl':
            backend = amcl_container
        elif mode == 'slam':
            backend = (slam_with_map_container
                       if LaunchConfiguration('load_map').perform(context) == 'true'
                       else slam_no_map_container)
        else:
            backend = Node(package='sentry_localization', executable='localization_backend',
                           arguments=['none'], output='screen')
        switched = False
        def fix(sub):
            return TextSubstitution(text=sub.perform(context))
        mapping = [slam_container(None, False, mapping=True, fix=fix),
                   map_pose('/pose', fix), map_saver(fix)]

        def backend_exited(event, context):
            nonlocal switched
            if context.is_shutdown or event.returncode != 42:
                return []
            switched = True
            if mode == 'none':
                return mapping
            return [EmitEvent(event=SignalProcess(
                signal_number=signal.SIGINT,
                process_matcher=lambda action: action is map_pose_node))]

        def pose_exited(event, context):
            return mapping if switched and not context.is_shutdown else []

        handlers = [RegisterEventHandler(OnProcessExit(
            target_action=backend, on_exit=backend_exited))]
        if mode != 'none':
            handlers += [RegisterEventHandler(OnProcessExit(
                target_action=map_pose_node, on_exit=pose_exited)), map_pose_node]
        return handlers + [backend]

    return LaunchDescription([
        DeclareLaunchArgument(
            'seed_from_raw_odom', default_value='false', choices=['true', 'false'],
            description='Wait up to 15 wall seconds for fresh raw /odom before starting '
            'EKF; use its x/y instead of initial_x/y. Requires use_rf2o:=true.'),
        DeclareLaunchArgument(
            'seed_require_team', default_value='false', choices=['true', 'false'],
            description='With seed_from_raw_odom, require referee robot_id then fresh '
            '/odom within 0.3 m of its team start (+/-4.625, 0).'),
        DeclareLaunchArgument(
            'initial_x', default_value='0.0',
            description='Initial firmware navigation X for EKF, meters'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0',
            description='Initial firmware navigation Y for EKF, meters'),
        DeclareLaunchArgument('amcl_initial_x', default_value=LaunchConfiguration('initial_x'),
                              description='AMCL field X; independent of EKF navigation seed'),
        DeclareLaunchArgument('amcl_initial_y', default_value=LaunchConfiguration('initial_y'),
                              description='AMCL field Y; defaults to initial_y, '
                              'independent of EKF navigation seed'),
        DeclareLaunchArgument('amcl_initial_yaw', default_value='0.0',
                              description='Field root yaw, not chassis joint yaw, radians'),
        use_sim_time_arg,
        odom_frame_arg, load_map_arg, map_file_arg, localization_mode_arg,
        use_rf2o_arg, autosave_map_arg, map_save_dir_arg, map_save_period_s_arg,
        DeclareLaunchArgument('seed_explicit_team', default_value='', choices=['', 'red', 'blue'],
                              description='Explicit practice team; check seed position '
                              'without referee.'),
        passthrough_odom_node, ekf_input_node, ekf_node,
        scan_odom_node, OpaqueFunction(function=managed_backend),
    ])
