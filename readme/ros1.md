# ROS1 Build of QuanergyClientRos

## Install the appropriate version of ROS1 for your operating system
The commands below should be sufficient for most installs but reference the official documentation if there are issues.
Ubuntu 20.04 official documentation: [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
Ubuntu 18.04 official documentation: [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```
### Ubuntu 20.04
```
sudo apt install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```
### Ubuntu 18.04
```
sudo apt install ros-melodic-desktop-full
source /opt/ros/melodic/setup.bash
```
## Build Instructions
Clone the SDK and the ROS repository

```
mkdir -p ~/QuanergySystems/catkin_ws/src
cd ~/QuanergySystems/catkin_ws/src
git clone https://github.com/QuanergySystems/quanergy_client.git
git clone https://github.com/QuanergySystems/quanergy_client_ros.git
```
Build QuanergyClientRos

```
cd ~/QuanergySystems/catkin_ws
catkin_make_isolated
```
## Testing Build
In the instructions below, substitute the appropriate distrobution for `<distro>` (noetic for 20.04 and melodic for 18.04).

```
source /opt/ros/<distro>/setup.bash
source ~/QuanergySystems/catkin_ws/devel_isolated/setup.bash
roslaunch quanergy_client_ros client.launch host:=<hostname_or_ip>
```
In a separate terminal, the following commands will show the output rate you are getting from your sensor.
```
source /opt/ros/<distro>/setup.bash
rostopic hz /quanergy/points
```
It is also possible to run the node directly rather than using the launch file. This exposes more options but complicates running multiple instances simultaneously.
```
rosrun quanergy_client_ros client_node --help
```

## Optional Configuration
To add ROS environment configuration automatically to every future bash session
```
echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
echo "source ~/QuanergySystems/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
```
