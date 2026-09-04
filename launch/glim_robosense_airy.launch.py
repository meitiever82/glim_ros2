'''
Author: meitiever
Date: 2026-04-14 16:18:52
LastEditors: meitiever
LastEditTime: 2026-06-05 16:31:48
Description: content
'''
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    glim_pkg = FindPackageShare('glim')

    # 生成带日期时间的dump路径: ~/map_airy/YYYYMMDD_HHMMSS
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    default_dump_path = os.path.join(os.path.expanduser('~'), 'map_airy', timestamp)

    # Launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='config/casbot_mapping',
        description='Config directory path (relative to glim package or absolute path)'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logging'
    )

    dump_on_unload_arg = DeclareLaunchArgument(
        'dump_on_unload',
        default_value='false',
        description='Dump data when node exits'
    )

    dump_path_arg = DeclareLaunchArgument(
        'dump_path',
        default_value=default_dump_path,
        description='Path to save dump data (default: ~/map_airy/YYYYMMDD_HHMMSS)'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/rslidar_imu_data',
        description='IMU topic name'
    )

    points_topic_arg = DeclareLaunchArgument(
        'points_topic',
        default_value='/rslidar_points',
        description='Point cloud topic name'
    )

    # GLIM node
    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'debug': LaunchConfiguration('debug'),
            'dump_on_unload': LaunchConfiguration('dump_on_unload'),
            'dump_path': LaunchConfiguration('dump_path'),
        }],
        remappings=[
            ('imu', LaunchConfiguration('imu_topic')),
            ('points', LaunchConfiguration('points_topic')),
        ],
    )

    return LaunchDescription([
        config_path_arg,
        debug_arg,
        dump_on_unload_arg,
        dump_path_arg,
        imu_topic_arg,
        points_topic_arg,
        glim_node,
    ])
