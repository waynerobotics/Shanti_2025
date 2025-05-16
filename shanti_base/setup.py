from setuptools import setup
import os
from glob import glob

package_name = 'shanti_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # ✅ Ensures Python package discovery
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Shanti base package for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shanti_base_node = shanti_base.shanti_base_node:main',
        ],
    },
)
