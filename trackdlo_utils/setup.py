from setuptools import find_packages, setup

package_name = 'trackdlo_utils'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hayato Shimada',
    maintainer_email='info@85-store.com',
    description='trackdlo_perception: Visualization and parameter tuning tools',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking_test = trackdlo_utils.tracking_test:main',
            'collect_pointcloud = trackdlo_utils.collect_pointcloud:main',
            'mask_node = trackdlo_utils.mask:main',
            'tracking_result_img = trackdlo_utils.tracking_result_img_from_pointcloud_topic:main',
            'composite_view = trackdlo_utils.composite_view_node:main',
            'param_tuner = trackdlo_utils.param_tuner_node:main',
        ],
    },
)
