# Shanti_2025

Repository for IGVC 2025

#These packages need to be installed:
Make sure to also run
sudo apt update first.

```bash
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-nmea-msgs \
    ros-humble-mavros-msgs \
    ros-humble-rosbridge-server
```

#Install Python dependencies for AI/ML modules:
Make sure you have pip installed, then run:

```bash
pip install torch transformers
```

## Clone a Repository with Submodules

If you clone this repository, you'll need to initialize the submodules after cloning:

Clone the main repository:

```bash
git clone <main-repo-url>
```

Navigate into the repository:

```bash
cd <repository-name>
```

Initialize and update the submodules:

```bash
git submodule update --init --recursive
```

## Update Submodules

If you've already cloned the repository and want to update the submodules to the latest commit:

```bash
git submodule update --recursive --remote
```

To update a specific submodule:

```bash
cd <submodule-directory>
git pull origin <branch>
cd ..
git add <submodule-directory>
git commit -m "Update submodule to latest commit"
git push
```

## Localization

This section contains the modules and tools related to the localization of the robot, which allows it to determine its position in the environment.

## Navigation

This section contains the code and algorithms used for the navigation system of the robot, ensuring that it moves efficiently and safely.

## Perception

This section includes perception modules, which allow the robot to detect and recognize objects in its environment using sensors like cameras and LIDAR.

## Practical Robot Tools

Utilities and tools that assist in building and maintaining the robot’s functionality.

## Shanti_Base

The core base software for the Shanti robot platform, managing essential control and integration with other modules.
