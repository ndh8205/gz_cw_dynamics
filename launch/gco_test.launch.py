"""Launch GCO formation test world (LVLH frame, 50 m GCO around chief).

Usage:
  ros2 launch gz_cw_dynamics gco_test.launch.py            # GUI
  ros2 launch gz_cw_dynamics gco_test.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as RosNode


def generate_launch_description():
    pkg_cw = get_package_share_directory('gz_cw_dynamics')
    pkg_orbit_sim = get_package_share_directory('orbit_sim')

    world_file = os.path.join(pkg_cw, 'worlds', 'gco_test.sdf')

    orbit_models = os.path.join(pkg_orbit_sim, 'models')
    existing_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = orbit_models + (
        ':' + existing_resource if existing_resource else '')

    cw_install_root = os.path.dirname(os.path.dirname(pkg_cw))
    cw_plugin_dir = os.path.join(cw_install_root, 'lib')
    existing_plugin = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = cw_plugin_dir + (
        ':' + existing_plugin if existing_plugin else '')

    headless = LaunchConfiguration('headless')
    verbose  = LaunchConfiguration('verbose')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('verbose',  default_value='3'),

        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',      resource_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),

        # GUI (default).
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '--verbose', verbose, world_file],
            output='screen',
            condition=UnlessCondition(headless),
        ),

        # Headless (server only).
        ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', '--verbose', verbose, world_file],
            output='screen',
            condition=IfCondition(headless),
        ),

        # Chief propagator node (starts 3 s after gz sim so scene is up).
        TimerAction(
            period=3.0,
            actions=[
                RosNode(
                    package='gz_cw_dynamics',
                    executable='chief_propagator_node.py',
                    name='chief_propagator',
                    output='screen',
                ),
            ],
        ),
    ])
