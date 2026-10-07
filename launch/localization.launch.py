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
Launch sentry_localization's map/odom localization stack.

Two orthogonal axes control localization:
- localization_mode (slam/mapping/amcl/none) picks who owns map->odom --
  slam_toolbox, amcl, or nobody (none). This is the map layer.
- use_rf2o (bool, default true) picks who owns odom->root: raw passthrough
  of /odom (false), or rf2o's /scan_odom fused with /odom by ekf_node
  (true). This is independent of localization_mode
  -- use_rf2o:=true can be layered on top of any localization_mode, including
  none (the old map-free "ekf mode" configuration).
Result always published on /localization/odom regardless of backend;
under slam/mapping/amcl, map_pose_publisher also puts it in the map frame
on /localization/map_odom.
load_map:=true (default) loads map_file's saved pose graph at startup.
See README.md's localization_mode/use_rf2o tables/Notes section for detail.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
        'odom_frame', default_value='odom',
        description='Frame slam_toolbox/amcl/ekf_node treat as their '
        'drift-free reference, parent of base_frame.'
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
        description='Selects who owns map->odom -- see the module '
        'docstring for what each of slam/mapping/amcl/none '
        'actually launches. Independent of use_rf2o, which '
        'owns odom->root.'
    )
    use_rf2o_arg = DeclareLaunchArgument(
        'use_rf2o', default_value='true',
        description='Whether rf2o scan odometry is fused into odom->root '
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
    slam_toolbox_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping' or ('", localization_mode,
         "' == 'slam' and '", LaunchConfiguration('load_map'), "' == 'true')"]
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
    # running, per the module docstring.
    map_yaml_file = PythonExpression(
        ["'", LaunchConfiguration('map_file'), "' + '.yaml'"]
    )

    # FASTRTPS_DEFAULT_PROFILES_FILE forces UDP-only transport (no shared
    # memory) -- this node has been observed hanging in rcl_node_init/
    # FastDDS SharedMemTransport::CreateInputChannelResource on startup,
    # before rclpy.spin() even runs, once /dev/shm accumulates many stale
    # fastrtps_* segments from earlier SIGKILLed runs -- SIGINT/SIGTERM are
    # never handled because the hang is below the Python signal-check
    # point. Same fix as thornbots_pkg's pose_translator/odom_tf_broadcaster
    # (see config/fastdds_no_shm.xml).
    passthrough_odom_node = Node(
        package='sentry_localization',
        executable='passthrough_odom_publisher',
        name='passthrough_odom_publisher',
        output='screen',
        condition=IfCondition(passthrough_selected),
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={
            'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(
                pkg_share, 'config', 'fastdds_no_shm.xml'
            )
        },
    )

    def make_ekf(context):
        node = Node(
            package='robot_localization',
            executable='ekf_node',
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
                    'initial_state': [
                        float(context.launch_configurations['initial_x']),
                        float(context.launch_configurations['initial_y'])] + [0.0] * 13,
                },
            ],
        )

        return [node]

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
            # The chassis never rotates (ekf.yaml); without this rf2o's
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
            ["'", localization_mode, "' == 'mapping' and '",
             LaunchConfiguration('autosave_map'), "' == 'true'"])),
        parameters=[{
            'save_dir': LaunchConfiguration('map_save_dir'),
            'period_s': ParameterValue(
                LaunchConfiguration('map_save_period_s'), value_type=float),
        }],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        condition=IfCondition(amcl_selected),
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=IfCondition(amcl_selected),
        parameters=[
            amcl_params_file,
            {
                'use_sim_time': use_sim_time,
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'base_frame_id': 'root',
                'initial_pose.x': ParameterValue(
                    LaunchConfiguration('initial_x'), value_type=float),
                'initial_pose.y': ParameterValue(
                    LaunchConfiguration('initial_y'), value_type=float),
                'global_frame_id': 'map',
                'scan_topic': '/scan',
            },
        ],
    )

    # map_server/amcl are nav2 lifecycle nodes -- they start unconfigured
    # and inactive on their own; this brings both up automatically
    # instead of requiring a manual configure/activate service call.
    amcl_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=IfCondition(amcl_selected),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # Two variants of the same node (mirrors sim.launch.py's gz_sim/
    # gz_sim_headless split): map_file_name is only meaningful to
    # slam_toolbox when actually set, and launch Node parameter dicts are
    # static, so load_map:=false needs a version of this node that omits
    # the key entirely rather than passing it empty. Both are also gated
    # on localization_mode being slam/mapping -- not launched at all when
    # localization_mode is amcl/none.
    slam_toolbox_with_map_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(slam_toolbox_with_map_selected),
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': LaunchConfiguration('odom_frame'),
                'map_file_name': LaunchConfiguration('map_file'),
                'map_start_pose': [0.0, 0.0, 0.0],
                'mode': slam_toolbox_mode_param,
                'use_map_saver': ParameterValue(
                    mapping_selected, value_type=bool
                ),
                'use_lifecycle_manager': True,
            },
        ],
    )
    slam_toolbox_no_map_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(slam_toolbox_no_map_selected),
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': LaunchConfiguration('odom_frame'),
                # Always mapping: this variant only ever launches when
                # localization_mode:=mapping (see
                # slam_toolbox_no_map_selected above) -- there's no saved
                # map to localize against without load_map anyway.
                'mode': 'mapping',
                'use_map_saver': True,
                'use_lifecycle_manager': True,
            },
        ],
    )

    # slam_toolbox is a lifecycle node since Jazzy (2.8) and starts
    # unconfigured; same bring-up as amcl's lifecycle manager above.
    slam_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        condition=IfCondition(slam_toolbox_selected),
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['slam_toolbox'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'initial_x', default_value='0.0', description='Known initial field X, meters'),
        DeclareLaunchArgument(
            'initial_y', default_value='0.0', description='Known initial field Y, meters'),
        use_sim_time_arg,
        odom_frame_arg, load_map_arg, map_file_arg, localization_mode_arg,
        use_rf2o_arg, autosave_map_arg, map_save_dir_arg, map_save_period_s_arg,
        passthrough_odom_node, ekf_node,
        scan_odom_node, map_pose_node, map_autosaver_node,
        slam_toolbox_with_map_node, slam_toolbox_no_map_node,
        slam_lifecycle_manager_node,
        map_server_node, amcl_node, amcl_lifecycle_manager_node,
    ])
