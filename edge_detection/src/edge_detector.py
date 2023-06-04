import numpy as np
import cv2
import argparse

def perform_edge_detection(image_path, min_threshold, max_threshold, min_line_length, max_line_gap):
    # Load the image
    image = cv2.imread(image_path)
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Apply Canny edge detection
    edges = cv2.Canny(gray, min_threshold, max_threshold, apertureSize=3)
    
    # Perform Hough line detection
    lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, lines=np.array([]),
                            minLineLength=min_line_length, maxLineGap=max_line_gap)
    
    # Draw the detected lines on the image
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 3)
    
    return image

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Perform edge detection on an image.')
parser.add_argument('image_path', type=str, help='Path to the image file')
parser.add_argument('--min_threshold', type=int, default=50, help='Minimum threshold value')
parser.add_argument('--max_threshold', type=int, default=150, help='Maximum threshold value')
parser.add_argument('--min_line_length', type=int, default=100, help='Minimum line length')
parser.add_argument('--max_line_gap', type=int, default=80, help='Maximum line gap')
args = parser.parse_args()

# Perform edge detection with the provided arguments
result_image = perform_edge_detection(args.image_path, args.min_threshold, args.max_threshold,
                                      args.min_line_length, args.max_line_gap)

# Display the result
cv2.imshow('Edge Detection Result', result_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

