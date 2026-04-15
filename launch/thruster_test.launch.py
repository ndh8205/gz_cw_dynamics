"""Thruster unit-test launch (GUI).

Fire each thruster via ROS 2 Float32 topics:

  ros2 topic pub -1 /deputy/thruster/fx_plus/cmd  std_msgs/Float32 "{data: 1.0}"
  ros2 topic pub -1 /deputy/thruster/fx_minus/cmd std_msgs/Float32 "{data: 1.0}"
  ros2 topic pub -1 /deputy/thruster/fy_plus/cmd  std_msgs/Float32 "{data: 1.0}"
  ros2 topic pub -1 /deputy/thruster/fy_minus/cmd std_msgs/Float32 "{data: 1.0}"
  ros2 topic pub -1 /deputy/thruster/fz_plus/cmd  std_msgs/Float32 "{data: 1.0}"
  ros2 topic pub -1 /deputy/thruster/fz_minus/cmd std_msgs/Float32 "{data: 1.0}"

Or continuous:
  ros2 topic pub /deputy/thruster/fx_plus/cmd std_msgs/Float32 "{data: 1.0}" --rate 10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    pkg_cw = get_package_share_directory('gz_cw_dynamics')
    world_file = os.path.join(pkg_cw, 'worlds', 'thruster_test.sdf')

    cw_install_root = os.path.dirname(os.path.dirname(pkg_cw))
    cw_plugin_dir   = os.path.join(cw_install_root, 'lib')
    existing_plugin = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = cw_plugin_dir + (
        ':' + existing_plugin if existing_plugin else '')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '--verbose', '3', world_file],
            output='screen',
        ),
    ])
