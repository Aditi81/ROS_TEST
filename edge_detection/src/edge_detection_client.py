#!/usr/bin/env python3

import rospy
import sys
sys.path.append('/home/aditi/catkin_ws/src')
import cv2
import time
from edge_detection.srv import edgedetection
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def edge_detection_client(image_path, min_threshold, max_threshold, min_line_length, max_line_gap):
    rospy.wait_for_service('edge_detection')
    try:
        edge = rospy.ServiceProxy('edge_detection', edgedetection)
        
        # Read the image file
        image = read_image(image_path)
        
        # Get the image dimensions
        height, width, channels = image.shape
        
        # Create the service request
        req = edgedetection._request_class()
        req.image.header.stamp = rospy.Time.now()
        req.image.height = height
        req.image.width = width
        req.image.encoding = 'bgr8'
        req.image.is_bigendian = 0
        req.image.step = width * channels
        req.image.data = image.tostring()
        req.min_threshold = min_threshold
        req.max_threshold = max_threshold
        req.min_line_length = min_line_length
        req.max_line_gap = max_line_gap
        
        # Call the service
        response = edge(req)
        
        # Process the service response
        if response.success:
            # Display the result image
            result_image = process_result_image(response.result_image)
            display_result_image(result_image)
        else:
            print("Edge detection failed")
    
    except Exception as e:
        print("Service call failed:", e)
        
def read_image(image_path):
    # Implement the image reading logic based on your requirements
    return cv2.imread(image_path)

def process_result_image(result_image):
    # Implement the result image processing logic based on your requirements
    bridge = CvBridge()
    return bridge.imgmsg_to_cv2(result_image, "bgr8")

def display_result_image(result_image):
    # diaplay detected image
    cv2.imshow('Edge Detection Result', result_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node('edge_detection_client')
    
    # Get the image path from the command-line argument
    if len(sys.argv) < 2:
        print("Please provide the image path as a command-line argument")
        sys.exit(1)
    image_path = sys.argv[1]
    
    min_threshold = 50
    max_threshold = 150
    min_line_length = 100
    max_line_gap = 80
    edge_detection_client(image_path, min_threshold, max_threshold, min_line_length, max_line_gap)
