# Copyright 2026 Hayato Shimada
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Test that launch files parse without errors."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


class TestLaunchFiles(unittest.TestCase):
    """Verify launch files can be loaded and parsed."""

    def test_trackdlo_launch_exists(self):
        """Check trackdlo.launch.py exists."""
        bringup_dir = get_package_share_directory('trackdlo_bringup')
        launch_file = os.path.join(bringup_dir, 'launch', 'trackdlo.launch.py')
        self.assertTrue(os.path.isfile(launch_file))

    def test_camera_launch_exists(self):
        """Check camera.launch.py exists."""
        bringup_dir = get_package_share_directory('trackdlo_bringup')
        launch_file = os.path.join(bringup_dir, 'launch', 'camera.launch.py')
        self.assertTrue(os.path.isfile(launch_file))

    def test_launch_description_creation(self):
        """Verify a basic LaunchDescription can be created."""
        ld = LaunchDescription([
            DeclareLaunchArgument('segmentation', default_value='hsv'),
        ])
        self.assertIsNotNone(ld)


if __name__ == '__main__':
    unittest.main()
