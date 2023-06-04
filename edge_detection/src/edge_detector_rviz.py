#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def perform_edge_detection(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Canny edge detection
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    
    # Perform Hough line detection
    lines = cv2.HoughLinesP(edges, rho=1, theta=1*np.pi/180, threshold=100, minLineLength=100, maxLineGap=80)
    
    # Draw the detected lines on the image
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
    cv2.imwrite('Image_edge1.png',image)
    return image

def image_callback(msg):
    # Convert ROS image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    # Perform edge detection on the image
    result_image = perform_edge_detection(cv_image)
    
    # Publish the processed image
    edge_image_msg = bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
    edge_image_pub.publish(edge_image_msg)
    
    # Display the original and edge images
    cv2.imshow("Edge Detection Result", result_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node("edge_detection_node")
    
    # Initialize the CvBridge
    bridge = CvBridge()
    
    # Subscribe to the image topic
    image_topic = "/camera/color/image_raw" # Replace with the actual image topic
    rospy.Subscriber(image_topic, Image, image_callback)
    edge_image_pub = rospy.Publisher("/edge_image", Image, queue_size=10)
    # Spin the ROS node
    rospy.spin()
