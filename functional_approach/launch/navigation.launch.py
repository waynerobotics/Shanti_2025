from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='2.0',
        description='Lookahead distance for the pure pursuit controller (meters)'
    )
    
    safety_dist_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='1.0',
        description='Safety distance for obstacle avoidance (meters)'
    )
    
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
    
    use_gps_waypoints_arg = DeclareLaunchArgument(
        'use_gps_waypoints',
        default_value='false',
        description='Whether waypoints are in GPS coordinates (lat/lon) or map frame'
    )

    return LaunchDescription([
        # Launch arguments
        lookahead_arg,
        safety_dist_arg,
        gnss_pose_topic_arg,
        use_tf_arg,
        use_gps_waypoints_arg,
        
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
        ),
        
        # Waypoint follower node
        Node(
            package='functional_approach',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'safety_distance': LaunchConfiguration('safety_distance'),
                'use_gps_waypoints': LaunchConfiguration('use_gps_waypoints'),
            }],
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
