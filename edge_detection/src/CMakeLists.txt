## Declare the Python script(s) to be installed and run
cmake_minimum_required(VERSION 3.0.2)
project(edge_detection)

catkin_install_python(PROGRAMS
  edge_detection_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
