#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time
import math


class EquirectStitcherNode(Node):
    def __init__(self):
        super().__init__('equirect_stitcher_node')
        
        # Bridge to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        
        # Create subscribers for camera feeds and camera info
        self.front_sub = self.create_subscription(
            Image,
            'front_camera/image_raw', 
            self.front_callback,
            10)
        
        self.rear_sub = self.create_subscription(
            Image,
            'rear_camera/image_raw',
            self.rear_callback,
            10)
            
        self.front_info_sub = self.create_subscription(
            CameraInfo, 
            'front_camera/camera_info', 
            self.front_info_callback, 
            10)
        
        self.rear_info_sub = self.create_subscription(
            CameraInfo, 
            'rear_camera/camera_info', 
            self.rear_info_callback, 
            10)
        
        # Publisher for the stitched equirectangular image
        self.stitched_pub = self.create_publisher(
            Image,
            'equirectangular_image',
            10)
        
        # Store the latest images from each camera
        self.front_img = None
        self.rear_img = None
        
        # Camera parameters
        self.K_front = None  # Front camera matrix
        self.D_front = None  # Front distortion coefficients
        self.K_rear = None   # Rear camera matrix
        self.D_rear = None   # Rear distortion coefficients
        
        # Default camera parameters for fisheye cameras
        self.default_K = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.default_D = np.array([0.5, 0.3, 0.1, 0.0])  # k1, k2, k3, k4 - Gazebo parameters
        
        # Output dimensions for equirectangular image
        self.equirect_width = 1440
        self.equirect_height = 720
        
        # Pre-calculated mapping tables
        self.front_map = None
        self.rear_map = None
        
        # Timestamp tracking for synchronization
        self.front_timestamp = None
        self.rear_timestamp = None
        
        # Maximum allowed time difference between frames (in seconds)
        self.max_sync_delay = 0.1
        
        # Initialize with default maps
        self.init_default_maps()
        
        self.get_logger().info('Equirectangular Stitcher Node initialized with direct mapping approach')
    
    def init_default_maps(self):
        """Initialize mapping with default camera parameters"""
        if self.K_front is None:
            self.K_front = self.default_K.copy()
            self.D_front = self.default_D.copy()
        
        if self.K_rear is None:
            self.K_rear = self.default_K.copy()
            self.D_rear = self.default_D.copy()
        
        self.create_mapping_tables()

    def front_info_callback(self, msg):
        """Callback for front camera info messages"""
        try:
            self.K_front = np.array(msg.k).reshape(3, 3)
            self.D_front = np.array(msg.d)[:4]
            self.get_logger().info('Received front camera calibration information')
            self.create_mapping_tables()
        except Exception as e:
            self.get_logger().error(f'Error processing front camera info: {e}')
            self.K_front = self.default_K.copy()
            self.D_front = self.default_D.copy()
            self.create_mapping_tables()

    def rear_info_callback(self, msg):
        """Callback for rear camera info messages"""
        try:
            self.K_rear = np.array(msg.k).reshape(3, 3)
            self.D_rear = np.array(msg.d)[:4]
            self.get_logger().info('Received rear camera calibration information')
            self.create_mapping_tables()
        except Exception as e:
            self.get_logger().error(f'Error processing rear camera info: {e}')
            self.K_rear = self.default_K.copy()
            self.D_rear = self.default_D.copy()
            self.create_mapping_tables()

    def create_mapping_tables(self):
        """Create mapping tables directly from spherical coordinates to fisheye images"""
        self.get_logger().info('Creating new equirectangular mapping tables')
        
        equirect_h = self.equirect_height
        equirect_w = self.equirect_width
        
        # Create mappings for both front and rear halves
        self.front_map = np.zeros((equirect_h, equirect_w//2, 2), dtype=np.float32)
        self.rear_map = np.zeros((equirect_h, equirect_w//2, 2), dtype=np.float32)
        
        # Calculate pixel mappings for each output pixel
        for y in range(equirect_h):
            for x in range(equirect_w):
                # Convert equirectangular pixel coords to spherical coordinates
                # x: [0, equirect_w] maps to azimuth [0, 2π]
                # y: [0, equirect_h] maps to elevation [π/2, -π/2] (top to bottom)
                azimuth = 2.0 * math.pi * x / equirect_w
                elevation = math.pi * (0.5 - y / equirect_h)
                
                # Convert spherical to 3D cartesian coordinates on unit sphere
                x3d = math.cos(elevation) * math.cos(azimuth)
                y3d = math.cos(elevation) * math.sin(azimuth)
                z3d = math.sin(elevation)
                
                # Determine which camera to use (front or rear)
                # Front camera: azimuth in [0, π]
                # Rear camera: azimuth in [π, 2π]
                if 0 <= azimuth < math.pi:  # Front camera
                    # For front camera, looking at +z direction
                    # Project the 3D point onto the camera plane (z=1)
                    if z3d <= 0:  # Behind the camera, would create distortion
                        continue
                        
                    # Calculate normalized image coordinates
                    x_norm = x3d / z3d
                    y_norm = y3d / z3d
                    
                    # Apply fisheye distortion model 
                    r = math.sqrt(x_norm*x_norm + y_norm*y_norm)
                    if r < 1e-6:
                        theta = 0
                    else:
                        theta = math.atan(r)
                    
                    # Apply fisheye distortion coefficients
                    theta_d = theta * (1 + self.D_front[0]*theta**2 + self.D_front[1]*theta**4 + 
                                      self.D_front[2]*theta**6 + self.D_front[3]*theta**8)
                    
                    # Scale distorted points
                    if r < 1e-6:
                        scale = 1.0
                    else:
                        scale = theta_d / r
                    
                    x_dist = x_norm * scale
                    y_dist = y_norm * scale
                    
                    # Apply camera matrix to get pixel coordinates
                    fx = self.K_front[0, 0]
                    fy = self.K_front[1, 1]
                    cx = self.K_front[0, 2]
                    cy = self.K_front[1, 2]
                    
                    img_x = fx * x_dist + cx
                    img_y = fy * y_dist + cy
                    
                    # Store mapping for this pixel
                    if 0 <= x < equirect_w//2:
                        self.front_map[y, x % (equirect_w//2)] = [img_x, img_y]
                
                else:  # Rear camera
                    # For rear camera, looking at -z direction
                    # Rotate the 3D point 180 degrees around y-axis
                    x3d_rear = -x3d
                    y3d_rear = y3d
                    z3d_rear = -z3d
                    
                    if z3d_rear <= 0:  # Behind the camera, would create distortion
                        continue
                        
                    # Calculate normalized image coordinates
                    x_norm = x3d_rear / z3d_rear
                    y_norm = y3d_rear / z3d_rear
                    
                    # Apply fisheye distortion model
                    r = math.sqrt(x_norm*x_norm + y_norm*y_norm)
                    if r < 1e-6:
                        theta = 0
                    else:
                        theta = math.atan(r)
                    
                    # Apply fisheye distortion coefficients
                    theta_d = theta * (1 + self.D_rear[0]*theta**2 + self.D_rear[1]*theta**4 + 
                                      self.D_rear[2]*theta**6 + self.D_rear[3]*theta**8)
                    
                    # Scale distorted points
                    if r < 1e-6:
                        scale = 1.0
                    else:
                        scale = theta_d / r
                    
                    x_dist = x_norm * scale
                    y_dist = y_norm * scale
                    
                    # Apply camera matrix to get pixel coordinates
                    fx = self.K_rear[0, 0]
                    fy = self.K_rear[1, 1]
                    cx = self.K_rear[0, 2]
                    cy = self.K_rear[1, 2]
                    
                    img_x = fx * x_dist + cx
                    img_y = fy * y_dist + cy
                    
                    # Store mapping for this pixel
                    if equirect_w//2 <= x < equirect_w:
                        self.rear_map[y, x % (equirect_w//2)] = [img_x, img_y]
        
        # Convert to more efficient format for cv2.remap
        self.front_xmap = self.front_map[:, :, 0]
        self.front_ymap = self.front_map[:, :, 1]
        self.rear_xmap = self.rear_map[:, :, 0]
        self.rear_ymap = self.rear_map[:, :, 1]
        
        self.get_logger().info('Created new equirectangular mapping tables')

    def front_callback(self, msg):
        """Callback for the front camera images"""
        try:
            self.front_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.front_timestamp = time.time()
            self.try_stitch_and_publish()
        except CvBridgeError as e:
            self.get_logger().error(f'Front camera CV bridge error: {e}')

    def rear_callback(self, msg):
        """Callback for the rear camera images"""
        try:
            self.rear_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rear_timestamp = time.time()
            self.try_stitch_and_publish()
        except CvBridgeError as e:
            self.get_logger().error(f'Rear camera CV bridge error: {e}')

    def try_stitch_and_publish(self):
        """Attempt stitching when both images are synchronized"""
        if self.front_img is None or self.rear_img is None:
            return

        if (abs(self.front_timestamp - self.rear_timestamp) < self.max_sync_delay):
            self.process_and_publish()

    def process_and_publish(self):
        """Create and publish equirectangular image using direct mapping"""
        if (self.front_xmap is None or self.front_ymap is None or
            self.rear_xmap is None or self.rear_ymap is None):
            self.get_logger().warning('Mapping tables not ready')
            return

        try:
            # Create empty equirectangular image
            equirect = np.zeros((self.equirect_height, self.equirect_width, 3), dtype=np.uint8)
            
            # Remap front camera image to left side of equirectangular image
            front_half = cv2.remap(self.front_img, self.front_xmap, self.front_ymap, 
                                   interpolation=cv2.INTER_LINEAR, 
                                   borderMode=cv2.BORDER_CONSTANT)
            
            # Remap rear camera image to right side of equirectangular image
            rear_half = cv2.remap(self.rear_img, self.rear_xmap, self.rear_ymap, 
                                 interpolation=cv2.INTER_LINEAR, 
                                 borderMode=cv2.BORDER_CONSTANT)
            
            # Combine the two halves
            equirect[:, :self.equirect_width//2] = front_half
            equirect[:, self.equirect_width//2:] = rear_half
            
            # Publish the result
            msg = self.bridge.cv2_to_imgmsg(equirect, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'equirect_camera'
            self.stitched_pub.publish(msg)
            
            self.get_logger().debug('Published stitched equirectangular image')
        
        except Exception as e:
            self.get_logger().error(f'Error in stitching process: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = EquirectStitcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()