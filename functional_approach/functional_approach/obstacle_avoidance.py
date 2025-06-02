#!/usr/bin/env python3

import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

def avoid_obstacles(cloud_msg, steering_cmd, safety_dist=1.0):
    """
    Obstacle avoidance algorithm that adjusts steering based on point cloud data.
    
    Args:
        cloud_msg: PointCloud2 message (obstacles in /base_link frame)
        steering_cmd: Current steering command from path following (radians)
        safety_dist: Safety distance for obstacle detection (meters)
        
    Returns:
        adjusted_steering: Steering angle adjusted for obstacle avoidance (radians)
    """
    # Initialize sectors (0=free, 1=blocked)
    # 8 sectors of 45° each, covering 360° around the robot
    sectors = [0] * 8
    
    # Convert point cloud to list of points
    points = point_cloud2.read_points(
        cloud_msg, 
        field_names=("x", "y", "z"), 
        skip_nans=True
    )
    
    # Filter points to consider only those in front of the robot and within detection range
    # We're primarily concerned with the area ahead for steering adjustments
    max_detection_range = 5.0  # meters
    
    # 1. Populate sectors based on point distances
    for point in points:
        x, y, _ = point
        
        # Skip points behind the robot (negative x)
        if x < 0:
            continue
            
        # Calculate distance to point
        distance = np.hypot(x, y)
        
        # Skip points beyond detection range
        if distance > max_detection_range:
            continue
            
        # Calculate angle to point (in radians)
        angle = np.arctan2(y, x)
        
        # Convert angle to sector index (0-7)
        # Sector 0: [-22.5°, 22.5°), Sector 1: [22.5°, 67.5°), etc.
        sector_idx = int(((angle + 2*np.pi) % (2*np.pi)) / (np.pi/4))
        
        # Mark sector as blocked if point is within safety distance
        if distance < safety_dist:
            sectors[sector_idx] = 1
    
    # 2. Find the sector corresponding to the current steering direction
    # Convert steering angle to sector
    # steering_cmd is in radians, 0 is straight ahead, positive is left turn
    # We need to convert it to an equivalent sector index
    current_sector = int(((steering_cmd + 2*np.pi) % (2*np.pi)) / (np.pi/4))
    
    # 3. Find nearest free sector to original steering direction
    # If current sector is free, keep it
    if sectors[current_sector] == 0:
        best_sector = current_sector
    else:
        # Check neighboring sectors (closest first)
        best_sector = None
        min_diff = float('inf')
        
        for i in range(8):
            if sectors[i] == 0:  # If sector is free
                # Calculate angular difference (considering wrap-around)
                diff = min((i - current_sector) % 8, (current_sector - i) % 8)
                
                if diff < min_diff:
                    min_diff = diff
                    best_sector = i
        
        # If all sectors are blocked, choose the one with the most distant obstacle
        if best_sector is None:
            best_sector = current_sector  # Default to current
    
    # 4. Convert best sector to steering angle
    # Convert sector index to angle (center of sector)
    best_angle = (best_sector * np.pi/4) + np.pi/8
    
    # Ensure angle is in range [-pi, pi]
    if best_angle > np.pi:
        best_angle -= 2*np.pi
    
    # 5. Blend commands: 70% path following, 30% avoidance
    # Only blend if there is an obstacle
    if sectors[current_sector] == 1:
        adjusted_steering = steering_cmd * 0.7 + best_angle * 0.3
    else:
        adjusted_steering = steering_cmd
    
    # Ensure steering angle is within limits (±30 degrees)
    return np.clip(adjusted_steering, -0.5, 0.5)
