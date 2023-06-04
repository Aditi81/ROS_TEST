#!/usr/bin/env python3

import sys 
sys.path.append('/home/aditi/catkin_ws/src')
import rospy
from edge_detection.srv import edgedetection, edgedetectionResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def perform_edge_detection(req):
    # Convert the ROS image to a CV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
    
    # Extract the parameters from the service request
    min_threshold = req.min_threshold
    max_threshold = req.max_threshold
    min_line_length = req.min_line_length
    max_line_gap = req.max_line_gap
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Apply Canny edge detection
    edges = cv2.Canny(gray, min_threshold, max_threshold, apertureSize=3)
    
    # Perform Hough line detection
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, lines=np.array([]),
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    
    # Draw the detected lines on the image
    result_image = cv_image.copy()

    for line in lines:
       x1, y1, x2, y2 = line[0]
       cv2.line(result_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
    # Convert the result image back to a ROS image message

    result_msg = bridge.cv2_to_imgmsg(result_image, "bgr8")
    # Create the service response
    response = edgedetectionResponse()
    response.success = True
    response.result_image =  result_msg
    return response

def edge_detection_server():
    rospy.init_node('edge_detection_server')
    rospy.Service('edge_detection', edgedetection, perform_edge_detection)
    rospy.spin()

if __name__ == "__main__":
    edge_detection_server()

