# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'realsense_params.yaml')

    rviz = LaunchConfiguration('rviz')
    segmentation_mode = LaunchConfiguration('segmentation_mode').perform(context)
    trackdlo_seg_mode = 'external' if segmentation_mode == 'sam2' else segmentation_mode

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
            parameters=[params_file, {'segmentation_mode': trackdlo_seg_mode}],
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

        # --- SAM2 Segmentation (when segmentation_mode=sam2) ---
        Node(
            package='trackdlo_segmentation',
            executable='sam2_segmentation',
            name='sam2_segmentation',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('segmentation_mode', 'sam2'),
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
        DeclareLaunchArgument(
            'segmentation_mode', default_value='internal',
            description='Segmentation: internal (HSV), external, sam2',
            choices=['internal', 'external', 'sam2'],
        ),
        OpaqueFunction(function=_launch_setup),
    ])
