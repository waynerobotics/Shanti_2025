#!/usr/bin/env python3

import numpy as np
import math

def pure_pursuit(current_pose, waypoints, lookahead=2.0):
    """
    Pure Pursuit path following algorithm.
    
    Args:
        current_pose: tuple (x, y, yaw) in ENU coordinates
        waypoints: list of (x, y) tuples in ENU coordinates
        lookahead: lookahead distance in meters
        
    Returns:
        steering_angle: steering angle in radians, limited to ±30 degrees (±0.5 radians)
    """
    x, y, yaw = current_pose
    
    if not waypoints or len(waypoints) < 2:
        return 0.0
    
    # 1. Find the nearest waypoint and path segment
    min_dist = float('inf')
    nearest_idx = 0
    
    for i, point in enumerate(waypoints):
        dist = np.hypot(point[0] - x, point[1] - y)
        if dist < min_dist:
            min_dist = dist
            nearest_idx = i
    
    # Create a list of candidate points starting from the nearest point
    candidates = []
    idx = nearest_idx
    
    # We'll look ahead a bit in the path to find intercept points
    while idx < len(waypoints):
        candidates.append(waypoints[idx])
        idx += 1
    
    # If we're at the end of the path, add the last waypoint
    if len(candidates) == 0 and len(waypoints) > 0:
        candidates.append(waypoints[-1])
    
    # 2. Transform waypoints to vehicle frame
    cos_yaw = np.cos(-yaw)
    sin_yaw = np.sin(-yaw)
    
    # Rotation and translation matrix to convert from global to vehicle frame
    transformed_candidates = []
    for wp in candidates:
        # Translation
        dx = wp[0] - x
        dy = wp[1] - y
        
        # Rotation
        tx = dx * cos_yaw - dy * sin_yaw
        ty = dx * sin_yaw + dy * cos_yaw
        
        transformed_candidates.append((tx, ty))
    
    # 3. Find the intercept point at lookahead distance
    target_point = None
    
    # Check if any point is at the lookahead distance
    for i in range(len(transformed_candidates) - 1):
        p1 = transformed_candidates[i]
        p2 = transformed_candidates[i+1]
        
        # Skip if both points are behind the robot
        if p1[0] < 0 and p2[0] < 0:
            continue
        
        # Line segment: p1 to p2
        # Circle: center at (0,0) with radius = lookahead
        # Find intersection of line segment and circle
        
        x1, y1 = p1
        x2, y2 = p2
        
        # Vector from p1 to p2
        dx = x2 - x1
        dy = y2 - y1
        
        # Quadratic equation coefficients: at^2 + bt + c = 0
        a = dx*dx + dy*dy
        b = 2 * (x1*dx + y1*dy)
        c = x1*x1 + y1*y1 - lookahead*lookahead
        
        discriminant = b*b - 4*a*c
        
        # No intersection
        if a < 1e-6 or discriminant < 0:
            continue
        
        # Calculate intersection points
        t1 = (-b + np.sqrt(discriminant)) / (2*a)
        t2 = (-b - np.sqrt(discriminant)) / (2*a)
        
        # Check if intersection is within line segment
        if 0 <= t1 <= 1:
            ix = x1 + t1 * dx
            iy = y1 + t1 * dy
            # We prefer points in front of the vehicle
            if ix > 0:
                target_point = (ix, iy)
                break
        
        if 0 <= t2 <= 1:
            ix = x1 + t2 * dx
            iy = y1 + t2 * dy
            # We prefer points in front of the vehicle
            if ix > 0:
                target_point = (ix, iy)
                break
    
    # If no intersection found, use the furthest waypoint or return zero steering
    if target_point is None:
        if transformed_candidates:
            # Find the furthest waypoint in front of the vehicle
            front_points = [p for p in transformed_candidates if p[0] > 0]
            if front_points:
                # Get the furthest point within lookahead distance
                front_points.sort(key=lambda p: p[0])  # Sort by x (distance ahead)
                target_point = front_points[-1]
            else:
                return 0.0
        else:
            return 0.0
    
    # 4. Compute curvature and steering angle
    tx, ty = target_point
    
    # Avoid division by zero
    if abs(tx) < 1e-6:
        return 0.0
    
    # Compute curvature (k = 2y / (x² + y²))
    # For Ackermann steering, we simplify to (2y / L²) if L ≈ x
    curvature = 2 * ty / (tx*tx + ty*ty)
    
    # Convert curvature to steering angle (for small angles: delta ≈ L * curvature)
    # where L is the wheelbase length (usually around 1-3m for cars)
    # We'll assume a wheelbase of 1m for simplicity
    wheelbase = 1.0
    steering_angle = np.arctan(wheelbase * curvature)
    
    # 5. Limit steering angle to ±30 degrees (±0.5 radians)
    return np.clip(steering_angle, -0.5, 0.5)
