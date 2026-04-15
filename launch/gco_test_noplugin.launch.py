"""Diagnostic launch: GCO scene without CW plugin (GUI freeze isolation)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    pkg_cw = get_package_share_directory('gz_cw_dynamics')
    pkg_orbit_sim = get_package_share_directory('orbit_sim')

    world_file = os.path.join(pkg_cw, 'worlds', 'gco_test_noplugin.sdf')
    orbit_models = os.path.join(pkg_orbit_sim, 'models')
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = orbit_models + (':' + existing if existing else '')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '--verbose', '3', world_file],
            output='screen',
        ),
    ])
