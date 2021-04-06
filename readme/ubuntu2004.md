# Ubuntu 20.04 Build of QuanergyClientRos

## Install QuanergyClient (SDK)

If you haven't already, build QuanergyClient (SDK) per the provided instructions (readme/ubuntu1804.md - instructions are identical for 20.04 and 18.04)
Install QuanergyClient

```
cd ~/QuanergySystems/quanergy_client/build
sudo make install
```
## Install ROS Noetic RVIZ and configure environment. [official documentation](http://wiki.ros.org/noetic/Installation/Ubuntu)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-rviz ros-noetic-pcl-ros ros-noetic-rosbash
source /opt/ros/noetic/setup.bash
```
## Build Instructions
Clone the ROS SDK repository

```
mkdir -p ~/QuanergySystems/catkin_ws/src
cd ~/QuanergySystems/catkin_ws/src
git clone https://github.com/QuanergySystems/quanergy_client_ros.git
```
Build QuanergyClientRos

```
cd ~/QuanergySystems/catkin_ws
catkin_make
```
## Testing Build

```
source /opt/ros/noetic/setup.bash
source ~/QuanergySystems/catkin_ws/devel/setup.bash
roslaunch quanergy_client_ros client.launch host:=<hostname_or_ip>
```
In a separate terminal, the following commands will show the output rate you are getting from your sensor.
```
source /opt/ros/noetic/setup.bash
rostopic hz /quanergy/points
```


## Optional Configuration
To add ROS environment configuration automatically to every future bash session
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/QuanergySystems/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
