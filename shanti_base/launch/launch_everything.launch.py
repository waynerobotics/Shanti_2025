#!/usr/bin/env python3

import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='perception',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='navigation',
            executable='navigation_node',
            name='navigation_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='localization',
            executable='localization_node',
            name='localization_node',
            output='screen'
        ),
        # If your simulation needs to start
        launch_ros.actions.Node(
            package='simulation',
            executable='simulation_node',
            name='simulation_node',
            output='screen'
        ),
    ])
