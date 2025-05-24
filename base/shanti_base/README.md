# Shanti Base

## Prerequisites

Download the test rosbag:
# Download location for controllerIn rosbag:
[ControllerIn](https://waynestateprod.sharepoint.com/:f:/s/WayneRoboticsTeam/EiBUklCwZBlCsRwHPR6T85MBHE-HUIELVuMF7MeLl3LKZA)


## Usage
```bash
ros2 launch shanti_base test_launch.xml
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| twist_topic | turtle1/cmd_vel | Topic name for turtle velocity commands |
| linear_scale | 2.0 | Scaling factor for linear velocity |
| angular_scale | 2.0 | Scaling factor for angular velocity |
| bag_path | $(env HOME)/rosbags/controllerIn | Path to the rosbag file |
