# Quanergy Sensor ROS SDK
ROS driver wrapping the QuanergyClient library to produce ROS point cloud messages from Quanergy sensors. 

This repo supports both ROS1 and ROS2 and has been tested on Ubuntu 18.04, 20.04, and 22.04.

Roslaunch files are provided that use the client.xml settings file that is in the settings folder. They provide host, ns, and topic arguments for avoiding collisions with multiple sensors.

The client_node can also be run directly with all options controllable at the command line. 

In either case, RVIZ can be used to view the point clouds.
## Build Instructions
[ROS2](readme/ros2.md)

[ROS1](readme/ros1.md)