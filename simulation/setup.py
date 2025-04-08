from setuptools import setup
import os
from glob import glob

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # ✅ Ensures Python package discovery
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Include launch files if any
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Simulation package for ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_node = simulation.simulation_node:main',  # ✅ Correct reference
            'equirect_stitcher = simulation.equirect_stitcher:main',  # ✅ Correct reference
        ],
    },
)
