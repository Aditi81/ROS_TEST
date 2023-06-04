## Tasks and results
**Folder structure**
```bash
└── edge_detection
    ├── CMakeLists.txt
    ├── data
    │   ├── edge1.png
    │   ├── edge2.png
    │   ├── edge3.png
    │   ├── edge4.png
    │   └── edge5.jpg
    ├── include
    │   └── edge_detection
    ├── launch
    │   ├── edge_detection.launch
    │   └── edge_detection_server.launch
    ├── msg
    │   └── Image.msg
    ├── package.xml
    ├── src
    │   ├── CMakeLists.txt
    │   ├── edge_detection_2d_3d.py
    │   ├── edge_detection_client.py
    │   ├── edge_detection_server.py
    │   ├── edge_detector.py
    │   ├── edge_detector_rviz.py
    │   └── __init__.py
    └── srv
        └── edgedetection.srv
```

* # **Basic: Image Edge Detection using Python and OpenCV**

  This script performs edge detection on an input image using the Canny edge detection algorithm and Hough line detection in OpenCV. It displays the output image with detected edges shown as green lines.

  ## a. Installation
  ``` 
      # Clone the repository:
      git clone https://github.com/your-username/edge-detection.git
      cd edge-detection
  ```
  ## b. Install the required Python packages:
  ```
      pip install -r requirements.txt
      numpy==1.21.0
      opencv-python==4.5.3.56

  ```

  ## c. Usage

  Run the `edge_detection_script.py` file with the desired input image and optional parameters for edge detection:
  ```
      python edge_detector.py image.jpg --min_threshold 50 --max_threshold 150 --min_line_length 100 --max_line_gap 80
  ```

  Replace `image.jpg` with the path to your input image file. Adjust the values of the edge detection parameters by specifying the optional arguments in the command.

  The script will perform edge detection on the image using the provided parameter values and display the resulting image with the detected edges shown as green lines.

  ## d. Concepts Used

  - Canny edge detection algorithm
  - Hough line detection
  - Command-line arguments parsing

  ## e. Possible Improvements

  - Implementing parameter tuning techniques to automatically find optimal threshold and line detection parameters for different images.
  - Adding support for different edge detection algorithms or additional image processing techniques like gaussian/median/sobel filter.
  - Building a graphical user interface for easier image selection and parameter adjustment.

* ## **Vision_ROS**

  ## a. Create a ROS Workspace
    ```
      mkdir -p ~/catkin_ws/src
      cd ~/catkin_ws/
      catkin_make
    ```
  ## b. Source catkin workspace
    ```
      echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
      source ~/.bashrc
    ```
  ## c. Start the ROS core:
    ```
      roscore
    ```

  * ## **Task 1: service with a client to detect edges**
    Provide ROS .srv and .msg files required to create a ROS service for edge detection. Give example usage of this service with a client to detect edges for image files in a directory.
    * Launch the edge detection server:
    Once the ROS core is running, you can proceed with launching the edge detection service and utilizing the provided client to detect edges for image files in a directory.
    ```
        roslaunch edge_detection edge_detection_server.launch
    ```  
    * Run the edge detection client with the image directory
     ```
        rosrun edge_detection edge_detection_client.py /path/to/image/directory
     ```
       Replace "/path/to/image/directory" with the actual directory path containing the image files you want to detect edges for.

    * The client will process each image file in the specified directory, send a service request to the edge detection server, and receive the edge detection results. The results will be displayed with green lines drawn on the detected edges.

  * ##  **Task 2: usage of ROS Edge Detection with RViz Visualization**
    This package provides functionality to detect edges for images subscribed from an image topic and visualize the detected edges on RViz. The images are obtained from a ROS bag file during playback. The detected edges are displayed as green lines overlaid on the original image.
    * Topis used 
        * image_topic = "/camera/color/image_raw"

    * Start the RViz application by running the following command:
     ```
         rosrun rviz rviz
     ```
    * In the RViz interface, add an `Image` display by clicking on the "Add" button and selecting `Image` from the menu.
    * Before launching the edge detection node, add **path of the bag file** containing the desired image topic in `edge_detector.launch`
    * Configure the Image display by setting the following parameters:
    * Launch the edge detection node and RViz configuration by running the following command
     ```
        roslaunch edge_detection edge_detector.launch
     ```
    * In RViz, select the image topic you want to display by clicking on the "Image" drop-down menu in the Image display and choosing the desired topic (e.g., /edge_image).
    * The detected edges will be visualized in RViz. The detected edges will appear as green lines overlaid on the input image.
  * ## **Task 3: Convert detected edge from 2D to 3D**
    This package provides functionality to detect edges in images obtained from a ROS bag file and convert the detected edge pixels from pixel coordinates (u, v) to 3D data (x, y, z) using depth images  and camera parameters. The 3D data is published to a ROS topic of type sensor_msgs/PointCloud2.
     * ### Topic used 
       * depth_image_topic = "/camera/depth/image_rect_raw"
       * camera_info_topic = "/camera/depth/camera_info" 

     * ### build a package
      ```
         catkin_make
      ```
     * During the execution, play the ROS bag file containing the desired image and depth topics by running the following command:
      ``` 
         rosbag play <bag_file.bag>
      ``` 
       Replace **<bag_file.bag>** with the actual path to your ROS bag file.

     * Start the edge detection and 2D-3D conversion node by running the following command:
      ```
         rosrun edge_detection edge_detector_2d_3d.py
      ```
     * In RViz, add a PointCloud2 display by clicking on the "Add" button and selecting **PointCloud2** from the menu.

     * Configure the PointCloud2 display by setting the following parameters:

         * Topic: Select the `/edge_points` topic.
         * Size (Pixels): Set an appropriate point size for visualization.
         * Color Transformer: Select the desired color transformation for the point cloud.
     * The input image and the detected edges will be visualized in RViz. The converted 3D point cloud data will also be displayed as a point cloud.


  * ## **Additional Notes**

    * Make sure to properly configure the image topic in RViz based on your specific setup.
    * Adjust the image topic and other parameters in the launch file and code as needed to match your ROS topic names and desired edge detection settings.
