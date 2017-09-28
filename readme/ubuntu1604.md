# quanergy_client_ros Build Procedure
ROS driver using quanergy_client to produce ROS point cloud messages from Quanergy sensors

# Linux Build Instructions (tested on Ubuntu 14.04 LTS)

Build quanergy_client (SDK) per README
Install quanergy_client (SDK)

```
cd ~/QuanergySystems/quanergy_client/build
sudo make install
```
Install ROS Indigo RVIZ and configure environment

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-rviz ros-indigo-pcl-ros
source /opt/ros/indigo/setup.bash
```
Clone the ROS SDK repository

```
mkdir -p ~/QuanergySystems/catkin_ws/src
cd ~/QuanergySystems/catkin_ws/src
catkin_init_workspace
git clone https://github.com/QuanergySystems/quanergy_client_ros.git
```
Build quanergy_client_ros code

```
cd ~/QuanergySystems/catkin_ws
catkin_make
```
To test, configure environment and launch node

```
source /opt/ros/indigo/setup.bash
source ~/QuanergySystems/catkin_ws/devel/setup.bash
roslaunch quanergy_client_ros client.launch host:=<hostname_or_ip>
```
To add ROS environment configuration automatically to every future bash session
```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
echo "source ~/QuanergySystems/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
