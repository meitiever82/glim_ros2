"""Localization on a prior map for Livox MID360 (tightly-coupled, via prior_map_localization glim_ext).

GLIM runs odometry-only (enable_local/global_mapping=false) and the prior_map_localization
extension injects Scan-to-Prior-Map VGICP factors into the odometry sliding-window graph,
publishing a `map -> odom` correction TF.

Prerequisites (handled by the `config/mid360_localization` config dir):
  - config_ros.json: enable_local/global_mapping=false, map_frame_id="" (GLIM must NOT publish
    map->odom — the extension does), extension_modules contains "libprior_map_localization.so".
  - config_prior_map_localization.json: set `prior_map_path` (or pass the prior_map_path: launch arg).

Give the initial pose with RViz "2D Pose Estimate" (publishes /initialpose), or set
has_default_init_pose + default_init_pose in config_prior_map_localization.json.

Example:
  ros2 launch glim_ros glim_mid360_localization.launch.py prior_map_path:=/data/tunnel.pcd use_rviz:=true
"""

import os
import re
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _resolve_config_dir(config_path: str) -> str:
    if config_path.startswith('/'):
        return config_path
    return os.path.join(get_package_share_directory('glim'), config_path)


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration('config_path').perform(context)
    prior_map_path = LaunchConfiguration('prior_map_path').perform(context)

    # Convenience: if prior_map_path is given, write it into the config dir's
    # config_prior_map_localization.json (GLIM has no ROS param to override config values).
    if prior_map_path:
        cfg_dir = _resolve_config_dir(config_path)
        cfg_file = os.path.join(cfg_dir, 'config_prior_map_localization.json')
        if os.path.isfile(cfg_file):
            with open(cfg_file, 'r', encoding='utf-8') as f:
                text = f.read()
            text, n = re.subn(r'("prior_map_path"\s*:\s*)"[^"]*"',
                              r'\1"' + prior_map_path.replace('\\', '\\\\') + '"', text, count=1)
            with open(cfg_file, 'w', encoding='utf-8') as f:
                f.write(text)
            print(f'[glim_mid360_localization] set prior_map_path={prior_map_path} in {cfg_file} ({n} replaced)')
        else:
            print(f'[glim_mid360_localization] WARNING: {cfg_file} not found; ignoring prior_map_path arg')

    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            # 非空时覆盖 config_ros.json 中的话题 (remapping 无法匹配 json 中的绝对话题名)
            'imu_topic': LaunchConfiguration('imu_topic'),
            'points_topic': LaunchConfiguration('points_topic'),
            'debug': LaunchConfiguration('debug'),
            'dump_on_unload': False,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('glim_ros'), 'rviz', 'glim_ros.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return [glim_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path', default_value='config/mid360_localization',
            description='Config directory (relative to glim package share, or absolute)'),
        DeclareLaunchArgument(
            'prior_map_path', default_value='',
            description='Prior map PCD path. If set, written into config_prior_map_localization.json.'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu', description='IMU topic'),
        DeclareLaunchArgument('points_topic', default_value='/livox/lidar', description='PointCloud2 topic'),
        DeclareLaunchArgument('debug', default_value='false', description='Enable debug logging'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz2'),
        OpaqueFunction(function=launch_setup),
    ])
