# Quanergy Sensor ROS SDK
ROS driver wrapping the QuanergyClient library to produce ROS point cloud messages from Quanergy sensors. 

A roslaunch file is provided that uses the client.xml settings file that is in the settings folder. It provides host, ns, and topic arguments for avoiding collisions with multiple sensors.

rosrun can also be used to run client_node directly with all options controllable at the command line. 

In either case, RVIZ can be used to view the point clouds.
## Build Instructions
[Ubuntu 20.04 LTS](readme/ubuntu2004.md)

[Ubuntu 18.04 LTS](readme/ubuntu1804.md)
