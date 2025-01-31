import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='shanti_base').find('shanti_base')
    default_model_path = os.path.join(pkg_share, 'src/description/shanti_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # Find Gazebo files
    gazebo_share = '/opt/ros/humble/share/gazebo_ros'
    gzclient_launch_path = os.path.join(gazebo_share, '/opt/ros/humble/share/gazebo_ros/launch/gzclient.launch.py')
    gzserver_launch_path = os.path.join(gazebo_share, '/opt/ros/humble/share/gazebo_ros/launch/gzserver.launch.py')

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch_path)
    )

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch_path),
        launch_arguments={
            'world': '/home/shanti/shanti2025/src/Shanti_2025/simulation/worlds/map10.world',
        }.items()
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
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
        arguments=['-entity', 'shanti', '-topic', 'robot_description'],
        output='screen'
    )

    joy_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node'
    )

    # Launch joystick axis to twist message conversion node
    joy2twist_node = launch_ros.actions.Node(
        package='joystick2base',
        executable='joy2twist',
        
        # Remap to Gazebo diff drive topic
        remappings=[
            ('/turtle1/cmd_vel', '/demo/cmd_vel')
        ] 
    )

    return launch.LaunchDescription([
        #launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #                                    description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),

        joint_state_publisher_node,
        #joint_state_publisher_gui_node... if running rviz
        robot_state_publisher_node,
        gzclient_launch,
        gzserver_launch,
        spawn_entity,
        joy_node,
        joy2twist_node,
        rviz_node
    ])
        
