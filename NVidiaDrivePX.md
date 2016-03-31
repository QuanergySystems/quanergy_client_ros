# Building quanergy_client_ros for Drive PX

#### Tested with Vibrante (Ubuntu 15.04 Vivid) on the Drive PX B00, and Ubuntu 14.04 Trusty Desktop

## Perform the following on the Drive PX

### Install Packages

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu vivid main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update

sudo apt-get install libconsole-bridge-dev libtinyxml-dev liblz4-dev libpoco-dev sip-dev uuid-dev \
    python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential git \
    libpcap-dev libboostall-dev libvtk5-dev libeigen3-dev libpcl-dev
```
### Build ROS from source (http://wiki.ros.org/indigo/Installation/Source)

#### Build workspace
```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws
```
#### Install ROS-com
```
rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --tar > indigo-ros_comm-wet.rosinstall
wstool init -j2 src indigo-ros_comm-wet.rosinstall
```
#### Install Perception_PCL
```
rosinstall_generator perception_pcl --rosdistro indigo --deps --wet-only --tar > indigo-ros_pcl-wet.rosinstall
wstool merge -t src indigo-ros_pcl-wet.rosinstall
wstool update -t src
```
#### Resolve Dependencies
```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro indigo -y
```
#### NOTE: Ignore the following error (the build in the next step still succeeds):
```
E: Package 'sbcl' has no installation candidate
ERROR: the following rosdeps failed to install
  apt: command [sudo -H apt-get install -y sbcl] failed
```
#### Build catkin workspace
```
./src/catkin/bin/catkin_make_isolated -j2 --install -DCMAKE_BUILD_TYPE=Release
```
#### Set environment variables to use ROS packages
```
source ~/ros_catkin_ws/install_isolated/setup.bash
```
### Build quanergy_client
#### Get code from Github
```
cd ~
git clone https://github.com/QuanergySystems/quanergy_client.git
cd ~/quanergy_client
```
#### Make build folder
```
mkdir build
cd build
```
#### Run cmake with option to not build visualizer
```
cmake -DNoViz=ON ..
```
#### NOTE: Ignore warnings
```
** WARNING ** io features related to openni will be disabled
** WARNING ** io features related to openni2 will be disabled
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
```
#### Build and install
```
make
sudo make install
```
### Build quanergy_client_ros
#### Get code from Github
```
cd ~
git clone https://github.com/QuanergySystems/quanergy_client_ros.git
```
#### Make build folder
```
cd ~/quanergy_client_ros
mkdir build
cd build
```
#### Run cmake
```
cmake ..
```
#### NOTE: Ignore warnings 
```
** WARNING ** io features related to openni will be disabled
** WARNING ** io features related to openni2 will be disabled
** WARNING ** io features related to pcap will be disabled
** WARNING ** io features related to png will be disabled
** WARNING ** io features related to libusb-1.0 will be disabled
```
#### Build and install
```
make
sudo make install
```
### Run quanergy_client_ros
#### Source setup file
```
source ~/quanergy_client_ros/build/devel/setup.bash
```
#### Set ROS_MASTER_URI and ROS_IP to IP address of Drive PX network interface
```
export ROS_MASTER_URI=http://<Drive PX network IP>:11311
export ROS_IP=<Drive PX network IP>
```
#### Launch quanergy_client_ros with sensor IP address
```
roslaunch quanergy_client_ros client.launch host:=<Sensor IP>
```

## Perform the following on the Ubuntu Desktop
### Install ROS (http://wiki.ros.org/indigo/Installation/Ubuntu)
#### Configure repositories
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
```
#### Install packages
```
apt-get install ros-indigo-ros-comm ros-indigo-diagnostics \
    ros-indigo-ros-base ros-indigo-nodelet ros-indigo-bfl ros-indigo-pcl-ros \
    ros-indigo-eigen-conversions ros-indigo-image-transport \
    ros-indigo-interactive-markers ros-indigo-laser-geometry \
    ros-indigo-map-msgs ros-indigo-media-export ros-indigo-python-qt-binding \
    ros-indigo-resource-retriever ros-indigo-urdf \
    ros-indigo-urdf-parser-plugin ros-indigo-image-geometry ros-indigo-rviz \
    ros-indigo-rqt-plot ros-indigo-rqt-rviz ros-indigo-cv-bridge
```
#### Initialize rosdep
```
sudo rosdep init
rosdep update
```
#### Setup environment
```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Run RViz
#### Set ROS_MASTER_URI to IP address of Drive PX on the network
```
export ROS_MASTER_URI=http://<Drive PX network IP>:11311
```
#### Set ROS_IP to IP of the Drive PX on the network
```
export ROS_IP=<Drive PX network IP>
```
#### Launch RViz
```
rviz &
```
#### In RViz, change Global Options → Fixed Frame to Sensor. Change PointCloud2 → Topic to /Sensor/points.
