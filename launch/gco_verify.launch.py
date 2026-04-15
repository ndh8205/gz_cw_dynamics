"""Headless GCO verification launch.

Spawns gz sim server-only on worlds/gco_verify.sdf (no camera/IMU, no mesh
meshes, RTF=100). Intended to be run alongside scripts/verify_gco.py.

    ros2 launch gz_cw_dynamics gco_verify.launch.py
    python3 ~/space_ros_ws/src/gz_cw_dynamics/scripts/verify_gco.py --orbits 3
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    pkg_cw = get_package_share_directory('gz_cw_dynamics')
    world_file = os.path.join(pkg_cw, 'worlds', 'gco_verify.sdf')

    cw_install_root = os.path.dirname(os.path.dirname(pkg_cw))
    cw_plugin_dir   = os.path.join(cw_install_root, 'lib')
    existing_plugin = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = cw_plugin_dir + (
        ':' + existing_plugin if existing_plugin else '')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', '--verbose', '3', world_file],
            output='screen',
        ),
    ])
