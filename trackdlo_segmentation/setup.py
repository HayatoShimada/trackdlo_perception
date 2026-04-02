from setuptools import find_packages, setup

package_name = 'trackdlo_segmentation'

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
    description='Pluggable segmentation interface for trackdlo_perception',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hsv_segmentation = trackdlo_segmentation.hsv_node:main',
        ],
    },
)
