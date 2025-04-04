# Shanti Rover Navigation

## Overview
This module provides navigation capabilities for the Shanti rover. It includes path planning, waypoint following, and obstacle avoidance functionality based on Nav2 (ROS2 Navigation).

## Components

### nav_bringup
The main navigation package that:
- Launches the Nav2 navigation stack configured for Shanti
- Provides waypoint navigation capabilities
- Handles GPS waypoint to local map conversions

#### Key Files:
- **nav_bringup.launch.py**: Main navigation launch file that starts all necessary components
- **waypoint_publisher.py**: Converts GPS waypoints to local map coordinates and sends them to the Nav2 waypoint follower
- **params/nav2_params.yaml**: Configuration for all Nav2 components including:
  - Planners
  - Controllers
  - Recovery behaviors
  - Costmaps
- **params/gps_waypoints.yaml**: Contains mission waypoints in GPS coordinates

## Usage

### Starting Navigation
To launch the navigation system:
```bash
ros2 launch nav_bringup nav_bringup.launch.py
```

### Providing Waypoints
Waypoints are defined in `nav_bringup/params/gps_waypoints.yaml` in the format:
```yaml
waypoints:
  - [latitude, longitude]  # Waypoint 1
  - [latitude, longitude]  # Waypoint 2
  ...
```

### Parameters

#### Controller
- **controller_frequency**: 20.0 Hz - How often the controller recalculates velocity commands
- **xy_goal_tolerance**: 1.0 m - How close the robot must be to a goal position to consider it reached
- **yaw_goal_tolerance**: 1.0 rad - How close the robot must be to a desired orientation to consider it reached

#### Planner
- **tolerance**: 0.5 m - How far from the goal position the plan can end
- **use_astar**: false - Whether to use A* algorithm for planning (false uses Dijkstra)
- **allow_unknown**: true - Whether the planner should plan through unknown areas

## Troubleshooting

### Common Issues

1. **Robot doesn't reach waypoints**:
   - Check if waypoint tolerances are appropriate for GPS accuracy
   - Verify the transform chain from map to UTM coordinates

2. **Navigation fails to start**:
   - Ensure all Nav2 nodes are properly started by the lifecycle manager
   - Check that required transforms (tf) are being published

3. **Path planning failures**:
   - Verify costmap parameters are appropriate for the environment
   - Check if obstacles are being properly detected and represented in costmaps

## Dependencies
- ROS2 Nav2 navigation stack
- UTM Python package (for GPS coordinate conversion)
- TF2 for coordinate transformations
- Localization system for providing odometry and GPS inputs