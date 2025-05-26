from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'differential_drive_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shanti',
    maintainer_email='blaine.oania@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = differential_drive_base.test:main',
            'encoder_odom = differential_drive_base.odometry_publisher:main',
            'roboclaw_controller = differential_drive_base.roboclaw_controller:main',
            'roboclaw_pwm_controller = differential_drive_base.roboclaw_pwm_controller:main',
        ],
    },
    py_modules=['differential_drive_base.roboclaw_3'],  # Explicitly include the roboclaw_3 module
)
