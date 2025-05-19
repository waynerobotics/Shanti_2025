#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import struct

class LaneSegmentationToPointCloud(Node):
    def __init__(self):
        super().__init__('lane_segmentation_to_pointcloud')
        self.declare_parameter('camera_height', 5.0)
        self.declare_parameter('fx', 500.0)
        self.declare_parameter('fy', 500.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('camera_topic', '/camera/camera_sensor/image_raw')
        self.declare_parameter('output_topic', '/lane_pointcloud')

        self.h = self.get_parameter('camera_height').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, 
                                                  self.get_parameter('camera_topic').value,
                                                  self.image_callback, 10)
        self.pc_pub = self.create_publisher(PointCloud2,
                                            self.get_parameter('output_topic').value, 10)

        print ('starting lane to point cloud')

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # Simple lane segmentation by thresholding
        _, mask = cv2.threshold(img, 175, 255, cv2.THRESH_BINARY)
        print('mask shape:', mask.shape);
        
        # Get coordinates of white pixels
        ys, xs = np.where(mask > 175)
        if len(xs) == 0:
            return

        points = []
        for u, v in zip(xs, ys):
            x = (u - self.cx) / self.fx * self.h
            y = (v - self.cy) / self.fy * self.h
            z = 0.0  # assume flat ground

            points.append([x, y, z])

        if not points:
            return

        # Convert to PointCloud2
        cloud_msg = self.points_to_pointcloud2(points, msg.header.stamp, frame_id='map')
        self.pc_pub.publish(cloud_msg)

    def points_to_pointcloud2(self, points, stamp, frame_id="map"):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        data = b''.join([struct.pack('fff', *pt) for pt in points])

        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12
        pc2.row_step = 12 * len(points)
        pc2.data = data
        pc2.is_dense = True
        return pc2

def main(args=None):
    rclpy.init(args=args)
    node = LaneSegmentationToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
