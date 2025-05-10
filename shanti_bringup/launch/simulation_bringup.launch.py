import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch_ros.actions import Node  # Ensure this is imported
import os

def generate_launch_description():

    print ('**********main configuration***********')
    print ('assuming ros2 workspace is ros2_wp')
    #get the home directory
    home_dir = os.path.expanduser('~/')
    print(home_dir)
    #get the ros_distro..default foxy
    ros_distro = os.getenv('ROS_DISTRO', 'foxy')
    print(ros_distro)

    # Parameter for teleoperation
    teleop_arg = launch.actions.DeclareLaunchArgument(
        'teleop',
        default_value='false',
        description='Flag to enable teleoperation'
    )

    # get the urdf model file
    pkg_share = launch_ros.substitutions.FindPackageShare(package='shanti_base').find('shanti_base')
    default_model_path = os.path.join(pkg_share, 'description/shanti_6w_lidar_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz.rviz')
    print (default_model_path)
    # Find Gazebo files
    gazebo_share = f'/opt/ros/{ros_distro}/share/gazebo_ros'
    print (gazebo_share)
    gzclient_launch_path = os.path.join(gazebo_share, 'launch/gzclient.launch.py')
    gzserver_launch_path = os.path.join(gazebo_share, 'launch/gzserver.launch.py')
    print(gzclient_launch_path)
    print(gzserver_launch_path)
   # Find the world file
    #worldfile = f'{home_dir}ros2_ws/src/Shanti_2025/simulation/worlds/map1.world'
    worldfile = f'{home_dir}ros2_ws/src/Shanti_2025/simulation/worlds/competition.world'
    print (worldfile)
    print ('****************************')
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch_path)
    )

    #launch gazibo 
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_path),
        launch_arguments={
            'world': worldfile,
            # 'pause' : 'true'
        }.items()
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        # parameters = [{'robot_description': LaunchConfiguration('model')}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path], #Add this line
        #parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'shanti',
            '-topic', 'robot_description',
            '-x', '13', '-y', '35', '-z', '1',  # orig..x,y = 0.  Position (x, y, z)... oakland: '-x', '-20.26', '-y', '24.8', '-z', '3',
            '-R', '0.0', '-P', '0.0', '-Y', '1.58093'   # Orientation (roll, pitch, yaw) in radians
        ],
        output='screen'
    )   

    # Launch joystick node conditionally based on teleop parameter
    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        condition=launch.conditions.IfCondition(LaunchConfiguration('teleop'))
    )

    # Launch joystick axis to twist message conversion node conditionally
    joy2twist_node = launch_ros.actions.Node(
        package='joystick2base',
        executable='joy2twist',
        
        # Remap to Gazebo diff drive topic
        remappings=[
            ('/turtle1/cmd_vel', '/demo/cmd_vel')
        ],
        parameters=[{
            'linear_axis': 1, 
            'angular_axis': 0, # 0 for yoke, 3 for joystick
        }],
        condition=launch.conditions.IfCondition(LaunchConfiguration('teleop'))
    )
    # Launch a node that will record the joystick button presses as GPS coordinates
    # This node will log the GPS coordinates when the joystick button is pressed 
    joystick_gps_logger_node = launch_ros.actions.Node(
        package='nav_bringup',
        executable='waypoint_joystick_record',
        name='joystick_gps_logger',
        output='screen',
        parameters=[
            {'output_directory': f'{home_dir}/ros2_ws/src/Shanti_2025/navigation/nav_bringup/params'},
            {'joystick_button_index': 0}  # Adjust button index if needed
        ],
        condition=launch.conditions.IfCondition(LaunchConfiguration('teleop'))
    )

    #this node take odom, gps and imu nodes and outputs it in utm coordinates
    localization_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('localization_bringup'),
            'launch',
            'dual_ekf_navsat.launch.py'
        )
    )
    )
    
    nav2_bringup_node =  IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('nav_bringup'),
            'launch',
            'nav_bringup.launch.py'
        )
    )
    )
    
    #this node relays the /demo/cmd_vel to all the wheels
    relay_cmd_vel = launch_ros.actions.Node(
        package='simulation',
        executable='simulation_node'
    )
    
    # Add waypoint flags spawn node
    spawn_waypoint_flags = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('shanti_bringup'),
                'launch',
                'spawn_waypoint_flags.launch.py'
            )
        ),
        launch_arguments={}.items()
    )

    # Add a static transform publisher node
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['14.25258886627853', '34.49661302869208', '0', '0', '0', '1.58093', 'world', 'map'],
        name='static_transform_publisher_gazebo_world_to_map'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                           description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        teleop_arg,
        static_transform_publisher,  # Add the static transform publisher here
        gzclient_launch,
        gzserver_launch,
        joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,

        #joystick nodes and the gps logger nodes

        joy_node,
        joy2twist_node,
        joystick_gps_logger_node,

        rviz_node,
        localization_node,
        #nav2_bringup_node,  
        
        relay_cmd_vel,
        
        spawn_waypoint_flags
        ])


