#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2

depth_image = None
camera_info = None

def depth_image_callback(msg):
    global depth_image
    depth_image = msg
    if depth_image is not None and camera_info is not None:
        process_edge_points()

def camera_info_callback(msg):
    global camera_info
    camera_info = msg
    if depth_image is not None and camera_info is not None:
        process_edge_points()

def edge_detection(depth_image):
    # Convert the image to grayscale
    #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    bridge = CvBridge()
    depth_array = bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")
    normalized_depth = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    
    # Apply Canny edge detection
    edges = cv2.Canny(normalized_depth, 50, 150, apertureSize=3)
    
    # Find edge pixels
    edge_pixels = np.argwhere(edges != 0)
    
    # Convert the (u, v) coordinates to (x, y, z) coordinates
    edge_points = []
    for pixel in edge_pixels:
        u, v = pixel[1], pixel[0]
        # Convert (u, v) to (x, y, z) using depth image and camera parameters
        depth = depth_array[v, u]
        if np.isnan(depth) or depth <= 0.0:
            continue
        x = (u - camera_info.K[2]) * depth / camera_info.K[0]
        y = (v - camera_info.K[5]) * depth / camera_info.K[4]
        z = depth
        edge_points.append((x, y, z))

    return edge_points

def process_edge_points():
    global depth_image, camera_info

    # Convert depth image to numpy array
    bridge = CvBridge()
    depth_array = bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

    # Retrieve camera parameters
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]
    width = camera_info.width
    height = camera_info.height

    # Create PointCloud2 message
    point_cloud_msg = PointCloud2()
    point_cloud_msg.header = depth_image.header
    point_cloud_msg.height = height
    point_cloud_msg.width = width
    point_cloud_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    point_cloud_msg.is_bigendian = False
    point_cloud_msg.point_step = 12
    point_cloud_msg.row_step = width * 12
    point_cloud_msg.is_dense = True

    # Load the input image
    #image = cv2.imread("path_to_image.jpg") # Replace "path_to_image.jpg" with the actual path to the image
    
    # Perform edge detection on the image
    edge_points = edge_detection(depth_image)
    
    # Add points to the point cloud message
    point_cloud_msg.data = np.asarray(edge_points, dtype=np.float32).tostring()

    # Publish the point cloud message
    pub = rospy.Publisher("edge_points", PointCloud2, queue_size=10)
    pub.publish(point_cloud_msg)

if __name__ == "__main__":
    rospy.init_node("edge_points_publisher")

    # Subscribe to depth image and camera info topics
    depth_image_topic = "/camera/depth/image_rect_raw"
    camera_info_topic = "/camera/depth/camera_info"

    # Define the callback functions for the subscribed topics
    rospy.Subscriber(depth_image_topic, Image, depth_image_callback)
    rospy.Subscriber(camera_info_topic, CameraInfo, camera_info_callback)

    # Spin until shutdown
    rospy.spin()

