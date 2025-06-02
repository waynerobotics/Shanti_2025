# GPS + IMU Localization for Shanti Robot

This package provides a GPS and IMU-based localization system for the Shanti robot, allowing navigation in outdoor environments without requiring wheel odometry.

## Overview

The localization system uses:

1. **XSens IMU** - Provides orientation and angular velocity data
2. **GPS Receiver** - Provides position data
3. **NTRIP Client** - For GPS corrections if available
4. **Robot Localization** - EKF filter for sensor fusion

## Hardware Requirements

- XSens MTi IMU
- GPS receiver (NMEA output)
- NTRIP corrections (optional but recommended for better accuracy)

## Installation

Ensure you have all dependencies installed:

```bash
sudo apt-get install ros-humble-robot-localization
pip3 install utm
```

## Usage

To launch the GPS + IMU localization system:

```bash
ros2 launch localization_bringup gps_imu_localization.launch.py
```

### Launch Options

- `use_sim_time` (default: false) - Set to true if using in simulation
- `gps_topic` (default: /gps/fix) - The topic for GPS data
- `imu_topic` (default: /xsens/imu/data) - The topic for IMU data
- `log_level` (default: info) - Logging level

## Configuration

The system is configured using the `gps_imu_params.yaml` file in the config directory. Key parameters to adjust:

- `navsat_transform` section:
  - `magnetic_declination_radians` - Set based on your location
  - `yaw_offset` - Adjust if IMU's yaw doesn't align with magnetic north
  - `datum` - Optional starting coordinates (latitude, longitude, altitude)

- `ekf_filter_node` section:
  - Sensor fusion configuration for GPS and IMU data
  - Process noise parameters can be tuned for different accuracy requirements

## Debugging

To monitor the GPS data and localization output:

```bash
ros2 run localization_bringup gps_monitor
```

This will display:
- Raw GPS coordinates
- UTM conversions
- Filtered odometry position
- Transform information

## Integration with Navigation

This localization system publishes:
- `/odometry/filtered` - Primary robot odometry
- `/tf` transforms between map and base_link frames

You can use this with Nav2 by configuring your navigation stack to use `/odometry/filtered` as the source of odometry.

## Customization

If you need to adjust the position of your IMU or GPS relative to the robot base, modify the static transform publisher parameters in the launch file.

## Troubleshooting

### No Odometry Output
- Check if your GPS is publishing data on the expected topic
- Check if your IMU is properly connected and publishing data
- Check the magnetic declination setting for your location

### Poor Localization Accuracy
- Ensure your GPS has a good fix with sufficient satellites
- Consider using NTRIP corrections
- Adjust the process noise parameters in the configuration file

### Transform Issues
- Verify the IMU is properly mounted and the orientation is correct
- Check the static transforms in the launch file

## Reference

For more information on the robot_localization package and its configuration:
- [robot_localization wiki](https://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [NavSatTransform tutorial](https://docs.ros.org/en/noetic/api/robot_localization/html/navsat_transform_node.html)
