# Shanti Rover Localization

## Overview
The localization module provides position and orientation estimation for the Shanti rover by integrating multiple sensor sources, including GPS, IMU, and wheel odometry. It uses a dual Extended Kalman Filter (EKF) approach to produce reliable pose estimates in both local and global coordinate frames.

## Components

### localization_bringup
The main localization package that:
- Provides integration of GPS, odometry, and IMU data
- Handles coordinate transforms between UTM and map frames
- Manages sensor fusion via dual EKF

#### Key Files:
- **dual_ekf_navsat.launch.py**: Main launch file that starts the dual EKF system
- **utm_map_transform_publisher.py**: Publishes transforms between UTM (global) and map (local) coordinate frames
- **config/dual_ekf_navsat_params.yaml**: Configuration parameters for both EKFs

## Architecture

The localization system uses a dual EKF approach:

1. **Local Odometry EKF**:
   - Fuses wheel odometry with IMU data
   - Provides smooth local position estimation
   - Operates in the odom frame

2. **Global Pose EKF**:
   - Fuses local odometry with GPS data
   - Handles GPS jumps and provides globally consistent positioning
   - Operates in the map frame

3. **UTM Transform Publisher**:
   - Bridges between GPS coordinates (lat/lon) and UTM coordinates
   - Maintains the transform between UTM and map frames
   - Essential for GPS waypoint navigation

## Usage

### Starting Localization
To launch the localization system:
```bash
ros2 launch localization_bringup dual_ekf_navsat.launch.py
```

### Required Inputs
The localization system expects the following inputs:
- `/imu/data` - IMU measurements with orientation, angular velocity, and linear acceleration
- `/gps/fix` - GPS position data in the NavSatFix format
- `/odometry/wheel` - Wheel odometry data

### Outputs
The system produces:
- `/odometry/filtered/local` - Smoothed local odometry (odom frame)
- `/odometry/filtered/global` - Global position estimate (map frame)
- TF transforms between frames: base_link → odom → map → utm → earth

## Configuration

### EKF Parameters
Key parameters that can be adjusted in `dual_ekf_navsat_params.yaml`:

#### Local EKF
- **frequency**: Update rate for local odometry fusion
- **sensor_timeout**: How long to wait before considering sensor data stale
- **two_d_mode**: Constrains motion to 2D plane when true

#### Global EKF
- **frequency**: Update rate for global position estimation
- **use_navsat**: Enables GPS integration
- **yaw_offset**: Corrects for magnetic declination
- **magnetic_declination_radians**: Local magnetic declination

### Transform Parameters
- **frequency**: How often the UTM to map transform is published
- **delay**: Startup delay before publishing transforms
- **magnetic_declination_radians**: Used for converting between true north and magnetic north

## Troubleshooting

### Common Issues

1. **Poor position estimates**:
   - Check IMU calibration
   - Verify GPS has good satellite visibility
   - Ensure odometry scaling is correct

2. **Jump in position estimates**:
   - Adjust process noise parameters for GPS inputs
   - Check for GPS multipath issues in the environment

3. **Inconsistent heading**:
   - Verify IMU magnetometer calibration
   - Check the magnetic_declination_radians parameter

## Dependencies
- robot_localization package
- navsat_transform_node
- TF2
- UTM Python package
- sensor_msgs
- nav_msgs