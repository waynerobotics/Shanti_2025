import os
from setuptools import find_packages, setup
import glob as glob

package_name = 'nav_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob.glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blaine',
    maintainer_email='Blaine.Oania@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_publisher = nav_bringup.waypoint_publisher:main',
            'goal_listener = nav_bringup.goal_listener:main',
            'waypoint_joystick_record = nav_bringup.waypoint_joystick_record:main',
            'lane_segmentation_to_pointcloud = nav_bringup.lane_segmentation_to_pointcloud:main',
            
        ],
    },
)
