from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulation',
            executable='equirect_stitcher',
            name='equirect_stitcher',
            output='screen',
            parameters=[
                {
                    'max_sync_delay': 0.1,
                },
            ],
            remappings=[
                ('front_camera/image_raw', 'front_camera/image_raw'),
                ('rear_camera/image_raw', 'rear_camera/image_raw'),
                ('front_camera/camera_info', 'front_camera/camera_info'),
                ('equirectangular_image', 'equirectangular_image'),
            ]
        )
    ])