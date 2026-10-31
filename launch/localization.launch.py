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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        'localization_mode', default_value='slam',
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
        'adding the team offset to raw /odom. Independent of localization_mode -- '
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
    slam_toolbox_no_map_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping' and '",
         LaunchConfiguration('load_map'), "' == 'false'"]
    )
    map_selected = PythonExpression(
        ["'", localization_mode, "' != 'none'"]
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
                     'navigation_frame': LaunchConfiguration('odom_frame'),
                     'require_team': ParameterValue(
                         LaunchConfiguration('seed_require_team'), value_type=bool),
                     'explicit_team': ParameterValue(
                         LaunchConfiguration('seed_explicit_team'), value_type=str)}],
        additional_env={
            'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
                pkg_share, 'config', 'fastdds_no_shm.xml'
            )
        },
    )

    scan_navigation_node = Node(
        package='sentry_localization', executable='passthrough_odom_publisher',
        name='scan_navigation_publisher', output='screen', condition=IfCondition(ekf_selected),
        parameters=[{'use_sim_time': use_sim_time,
                     'navigation_frame': LaunchConfiguration('odom_frame'),
                     'require_team': ParameterValue(
                         LaunchConfiguration('seed_require_team'), value_type=bool),
                     'explicit_team': ParameterValue(
                         LaunchConfiguration('seed_explicit_team'), value_type=str)}],
        remappings=[('/odom', '/scan_odom'),
                    ('/localization/odom', '/localization/scan_odom')],
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
        arguments = ['--seed-use-sim-time', use_sim_time]
        if context.launch_configurations['seed_require_team'].lower() == 'true':
            arguments.append('--require-team')
        elif context.launch_configurations.get('seed_explicit_team', ''):
            arguments.extend(['--team', context.launch_configurations['seed_explicit_team']])
        node = Node(
            package='sentry_localization', executable='seeded_ekf',
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
                },
            ],
        )

        shutdown = RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='EKF process exited'))]))
        return [shutdown, node]

    ekf_node = OpaqueFunction(function=make_ekf)

    # Only used when use_rf2o:=true; the adapter adds the navigation field offset.
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
    map_pose_node = Node(
        package='sentry_localization',
        executable='map_pose_publisher',
        name='map_pose_publisher',
        output='screen',
        condition=IfCondition(map_selected),
        parameters=[{
            'use_sim_time': use_sim_time,
            'backend_pose_topic': backend_pose_topic,
        }],
    )

    map_autosaver_node = Node(
        package='sentry_localization',
        executable='map_autosaver',
        name='map_autosaver',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", localization_mode, "' == 'mapping'"])),
        parameters=[{
            'use_sim_time': use_sim_time,
            'periodic_enabled': ParameterValue(
                LaunchConfiguration('autosave_map'), value_type=bool),
            'save_dir': LaunchConfiguration('map_save_dir'),
            'period_s': ParameterValue(
                LaunchConfiguration('map_save_period_s'), value_type=float),
        }],
    )

    # Construct backend components locally; no load_node RPC can race discovery.
    amcl_container = Node(
        package='sentry_localization', executable='seeded_ekf',
        name='localization_container',
        arguments=['--seed-use-sim-time', use_sim_time, '--backend', 'amcl'], output='screen',
        condition=IfCondition(amcl_selected),
        parameters=[amcl_params_file, {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
            'odom_frame_id': LaunchConfiguration('odom_frame'),
            'base_frame_id': 'root',
            'global_frame_id': 'map',
            'scan_topic': '/scan',
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }])

    def slam_container(condition, load_map, mapping=False):
        params = {
            'use_sim_time': use_sim_time,
            'odom_frame': LaunchConfiguration('odom_frame'),
            'mode': slam_toolbox_mode_param,
            'use_map_saver': ParameterValue(mapping_selected, value_type=bool),
            'zero_origin_mapping': mapping and not load_map,
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
            package='sentry_localization',
            executable='localization_backend' if mapping else 'seeded_ekf',
            name='slam_container',
            arguments=['slam'] if mapping else [
                '--seed-use-sim-time', use_sim_time, '--backend', 'slam'], output='screen',
            condition=IfCondition(condition), parameters=[slam_params_file, params])

    slam_with_map_container = slam_container(PythonExpression([
        "'", localization_mode, "' == 'slam' and '",
        LaunchConfiguration('load_map'), "' == 'true'"]), True)
    mapping_with_map_container = slam_container(PythonExpression([
        "'", localization_mode, "' == 'mapping' and '",
        LaunchConfiguration('load_map'), "' == 'true'"]), True, mapping=True)
    slam_no_map_container = slam_container(slam_toolbox_no_map_selected, False, mapping=True)

    return LaunchDescription([
        DeclareLaunchArgument('seed_from_raw_odom', default_value='true',
                              description='Compatibility argument; navigation always seeds '
                              'from fresh raw /odom.'),
        DeclareLaunchArgument('seed_require_team', default_value='true', choices=['true', 'false'],
                              description='Wait for actual referee team and raw /odom near '
                              'zero, then add team start. False allows room navigation. '
                              'AMCL and saved-map SLAM still require actual referee.'),
        DeclareLaunchArgument('seed_explicit_team', default_value='', choices=['', 'red', 'blue'],
                              description='Practice navigation team only when seed_require_team '
                              'is false; never bypasses map localization referee gate.'),
        *[DeclareLaunchArgument(name, default_value='0.0',
                                description='Deprecated; initialization uses raw /odom and team.')
          for name in ['initial_x', 'initial_y', 'amcl_initial_x', 'amcl_initial_y',
                       'amcl_initial_yaw']],
        use_sim_time_arg,
        odom_frame_arg, load_map_arg, map_file_arg, localization_mode_arg,
        use_rf2o_arg, autosave_map_arg, map_save_dir_arg, map_save_period_s_arg,
        passthrough_odom_node, ekf_input_node, ekf_node,
        scan_odom_node, scan_navigation_node, map_pose_node, map_autosaver_node,
        slam_with_map_container, mapping_with_map_container, slam_no_map_container, amcl_container,
    ])
