# Ubuntu 18.04 Build of QuanergyClientRos

## Install ROS Melodic RVIZ and configure environment. [official documentation](http://wiki.ros.org/melodic/Installation/Ubuntu)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-rviz ros-melodic-pcl-ros ros-melodic-rosbash
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

```
source /opt/ros/melodic/setup.bash
source ~/QuanergySystems/catkin_ws/devel_isolated/setup.bash
roslaunch quanergy_client_ros client.launch host:=<hostname_or_ip>
```
In a separate terminal, the following commands will show the output rate you are getting from your sensor.
```
source /opt/ros/melodic/setup.bash
rostopic hz /quanergy/points
```


## Optional Configuration
To add ROS environment configuration automatically to every future bash session
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "source ~/QuanergySystems/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
```
