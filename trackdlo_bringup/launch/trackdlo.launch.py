# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'realsense_params.yaml')

    rviz = LaunchConfiguration('rviz')

    return [
        # --- Static TF: base_link -> camera_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # --- TrackDLO C++ tracking node ---
        Node(
            package='trackdlo_core',
            executable='trackdlo',
            name='trackdlo',
            output='screen',
            parameters=[params_file],
        ),

        # --- Python initialization node ---
        Node(
            package='trackdlo_core',
            executable='init_tracker',
            name='init_tracker',
            output='screen',
            parameters=[params_file],
        ),

        # --- Composite View (4-panel display) ---
        Node(
            package='trackdlo_utils',
            executable='composite_view',
            name='composite_view',
            output='screen',
            parameters=[params_file],
        ),

        # --- CPD-LLE Parameter Tuner ---
        Node(
            package='trackdlo_utils',
            executable='param_tuner',
            name='param_tuner',
            output='screen',
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(bringup_dir, 'rviz', 'tracking.rviz')
            ],
            condition=IfCondition(rviz),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 for visualization',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
