from setuptools import setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # ✅ Ensures Python package discovery
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Perception package for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = perception.perception_node:main',  # ✅ Correct reference
        ],
    },
)
