from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    gnss_pose_topic_arg = DeclareLaunchArgument(
        'gnss_pose_topic',
        default_value='/gnss_pose',
        description='Topic for GNSS pose data (PoseStamped message with lat/lon coordinates)'
    )
    
    use_tf_arg = DeclareLaunchArgument(
        'use_tf',
        default_value='true',
        description='Whether to broadcast the transform from map to base_link'
    )
    
    return LaunchDescription([
        # Launch arguments
        gnss_pose_topic_arg,
        use_tf_arg,
        
        # Localization node
        Node(
            package='functional_approach',
            executable='gps_imu_localization',
            name='gps_imu_localization',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_tf': LaunchConfiguration('use_tf'),
            }],
            remappings=[
                ('/gnss_pose', LaunchConfiguration('gnss_pose_topic')),
            ],
        )
    ])

if __name__ == '__main__':
    generate_launch_description()