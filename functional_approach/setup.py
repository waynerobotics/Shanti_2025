from setuptools import find_packages, setup

package_name = 'functional_approach'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/localization.launch.py',
            'launch/navigation.launch.py',
        ]),
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
            'gps_imu_localization = functional_approach.gps_imu_localization:main',
            'waypoint_follower = functional_approach.waypoint_follower:main',
        ],
    },
)
