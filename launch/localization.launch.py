"""
Turns /odom + /scan into odom->root, and (for map-based backends)
map->odom -- scheme picked by localization_mode (slam/mapping/amcl/ekf).
Result always published on /localization/odom regardless of backend.
load_map:=true (default) loads map_file's saved pose graph at startup.
See README.md's localization_mode table/Notes section for per-mode detail.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_localization")
    slam_params_file = os.path.join(pkg_share, "config", "slam.yaml")
    ekf_params_file = os.path.join(pkg_share, "config", "ekf.yaml")
    amcl_params_file = os.path.join(pkg_share, "config", "amcl.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Forwarded from sentry_pkg/auto.launch.py's "
                     "real_hardware-derived value."
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    odom_frame_arg = DeclareLaunchArgument(
        "odom_frame", default_value="odom",
        description="Frame slam_toolbox/amcl/ekf_node treat as their "
                     "drift-free reference, parent of base_frame."
    )

    load_map_arg = DeclareLaunchArgument(
        "load_map", default_value="true",
        description="Deserialize map_file's saved pose graph at startup and "
                     "continue from it instead of starting blank. Only "
                     "affects localization_mode:=slam/mapping (and only "
                     "actually works for those modes against a map_file "
                     "that has a real .posegraph/.data, see map_file "
                     "below -- clean_map does not yet); amcl always loads "
                     "map_file's .yaml regardless, and ekf runs no map "
                     "node at all."
    )
    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value=os.path.join(pkg_share, "map", "clean_map"),
        description="Path (no extension) to the map to use: slam_toolbox "
                     "reads <map_file>.posegraph/.data (see "
                     "slam_toolbox/srv/SerializePoseGraph), amcl reads "
                     "<map_file>.yaml (see nav2_map_server). Same basename, "
                     "both refer to the same saved map. Default is "
                     "clean_map -- it only has a .yaml/.pgm (map_server-ready, "
                     "so localization_mode:=amcl works against it out of the "
                     "box), NOT a .posegraph/.data, so "
                     "localization_mode:=slam/mapping with load_map:=true "
                     "(both also defaults) will fail to deserialize "
                     "against it until a real mapping run produces one -- "
                     "pass map_file:=<pkg_share>/map/ARCC26 explicitly for "
                     "slam/mapping until then."
    )

    localization_mode_arg = DeclareLaunchArgument(
        "localization_mode", default_value="slam",
        choices=["slam", "mapping", "amcl", "ekf"],
        description="Selects the whole localization scheme in one choice "
                     "-- see the module docstring for what each of "
                     "slam/mapping/amcl/ekf actually launches and which "
                     "TF edges (map->odom, odom->root) it owns."
    )
    localization_mode = LaunchConfiguration("localization_mode")
    ekf_selected = PythonExpression(
        ["'", localization_mode, "' == 'ekf'"]
    )
    amcl_selected = PythonExpression(
        ["'", localization_mode, "' == 'amcl'"]
    )
    mapping_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping'"]
    )
    passthrough_selected = PythonExpression(
        ["'", localization_mode, "' in ('slam', 'mapping', 'amcl')"]
    )
    slam_toolbox_with_map_selected = PythonExpression(
        ["'", localization_mode, "' in ('slam', 'mapping') and '",
         LaunchConfiguration("load_map"), "' == 'true'"]
    )
    slam_toolbox_no_map_selected = PythonExpression(
        ["'", localization_mode, "' == 'mapping' and '",
         LaunchConfiguration("load_map"), "' == 'false'"]
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
        ["'", LaunchConfiguration("map_file"), "' + '.yaml'"]
    )

    # FASTRTPS_DEFAULT_PROFILES_FILE forces UDP-only transport (no shared
    # memory) -- this node has been observed hanging in rcl_node_init/
    # FastDDS SharedMemTransport::CreateInputChannelResource on startup,
    # before rclpy.spin() even runs, once /dev/shm accumulates many stale
    # fastrtps_* segments from earlier SIGKILLed runs -- SIGINT/SIGTERM are
    # never handled because the hang is below the Python signal-check
    # point. Same fix as sentry_pkg's pose_translator/odom_tf_broadcaster
    # (see config/fastdds_no_shm.xml).
    passthrough_odom_node = Node(
        package="sentry_localization",
        executable="passthrough_odom_publisher",
        name="passthrough_odom_publisher",
        output="screen",
        condition=IfCondition(passthrough_selected),
        parameters=[{"use_sim_time": use_sim_time}],
        additional_env={
            "FASTRTPS_DEFAULT_PROFILES_FILE": os.path.join(
                pkg_share, "config", "fastdds_no_shm.xml"
            )
        },
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(ekf_selected),
        remappings=[("odometry/filtered", "/localization/odom")],
        parameters=[
            ekf_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_link_frame": "root",
                # Must match odom_frame, not base_link_frame -- see
                # config/ekf.yaml's comment on world_frame.
                "world_frame": LaunchConfiguration("odom_frame"),
                "publish_tf": False,
            },
        ],
    )

    # Only used by localization_mode:=ekf; nothing else reads /scan_odom.
    scan_odom_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        condition=IfCondition(ekf_selected),
        parameters=[{
            "laser_scan_topic": "/scan",
            "odom_topic": "/scan_odom",
            "publish_tf": False,
            "base_frame_id": "root",
            "odom_frame_id": LaunchConfiguration("odom_frame"),
            "init_pose_from_topic": "",
            "freq": 20.0,
            "use_sim_time": use_sim_time,
        }],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        condition=IfCondition(amcl_selected),
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml_file,
        }],
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        condition=IfCondition(amcl_selected),
        parameters=[
            amcl_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame_id": LaunchConfiguration("odom_frame"),
                "base_frame_id": "root",
                "global_frame_id": "map",
                "scan_topic": "/scan",
            },
        ],
    )

    # map_server/amcl are nav2 lifecycle nodes -- they start unconfigured
    # and inactive on their own; this brings both up automatically
    # instead of requiring a manual configure/activate service call.
    amcl_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        condition=IfCondition(amcl_selected),
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    # Two variants of the same node (mirrors sim.launch.py's gz_sim/
    # gz_sim_headless split): map_file_name is only meaningful to
    # slam_toolbox when actually set, and launch Node parameter dicts are
    # static, so load_map:=false needs a version of this node that omits
    # the key entirely rather than passing it empty. Both are also gated
    # on localization_mode being slam/mapping -- not launched at all when
    # localization_mode is amcl/ekf.
    slam_toolbox_with_map_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(slam_toolbox_with_map_selected),
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
                "map_file_name": LaunchConfiguration("map_file"),
                "map_start_pose": [0.0, 0.0, 0.0],
                "mode": slam_toolbox_mode_param,
                "use_map_saver": ParameterValue(
                    mapping_selected, value_type=bool
                ),
            },
        ],
    )
    slam_toolbox_no_map_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=IfCondition(slam_toolbox_no_map_selected),
        parameters=[
            slam_params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": LaunchConfiguration("odom_frame"),
                # Always mapping: this variant only ever launches when
                # localization_mode:=mapping (see
                # slam_toolbox_no_map_selected above) -- there's no saved
                # map to localize against without load_map anyway.
                "mode": "mapping",
                "use_map_saver": True,
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_frame_arg, load_map_arg, map_file_arg, localization_mode_arg,
        passthrough_odom_node, ekf_node,
        scan_odom_node,
        slam_toolbox_with_map_node, slam_toolbox_no_map_node,
        map_server_node, amcl_node, amcl_lifecycle_manager_node,
    ])
