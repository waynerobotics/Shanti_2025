import os
from setuptools import find_packages, setup
import glob as glob

package_name = 'localization_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'utm'],
    zip_safe=True,
    maintainer='shanti',
    maintainer_email='blaine.oania@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'utm_map_transform_publisher = localization_bringup.utm_map_transform_publisher:main',
            'custom_lifecycle_manager = localization_bringup.custom_lifecycle_manager:main',
            'odometry_rebroadcaster = localization_bringup.odometry_rebroadcaster:main',
            'gps_monitor = localization_bringup.gps_monitor:main',
            'pose_to_gps_converter = localization_bringup.pose_to_gps_converter:main',
        ],
    },
)
