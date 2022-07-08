# ROS2 Build of QuanergyClientRos

## Install the appropriate version of ROS2 for your operating system
The commands below should be sufficient for most installs but reference the official documentation if there are issues.
Ubuntu 22.04 official documentation: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
Ubuntu 20.04 official documentation: [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

```
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
```
### Ubuntu 22.04
```
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```
### Ubuntu 20.04
```
sudo apt install ros-foxy-desktop
source /opt/ros/foxy/setup.bash
```
## Install [Colcon](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) for building packages
```
sudo apt install python3-colcon-common-extensions
```
## Build Instructions
Clone the SDK and the ROS repository

```
mkdir -p ~/QuanergySystems/sdk_ws/src
cd ~/QuanergySystems/sdk_ws/src
git clone https://github.com/QuanergySystems/quanergy_client.git
git clone https://github.com/QuanergySystems/quanergy_client_ros.git
```
Build QuanergyClientRos

```
cd ~/QuanergySystems/sdk_ws
colcon build
```
## Testing Build
In the instructions below, substitute the appropriate distrobution for `<distro>` (humble for 22.04 and foxy for 20.04).

```
source /opt/ros/<distro>/setup.bash
source ~/QuanergySystems/sdk_ws/install/setup.bash
ros2 launch quanergy_client_ros client.launch.xml host:=<hostname_or_ip>
```
In a separate terminal, the following commands will show the output rate you are getting from your sensor.
```
source /opt/ros/<distro>/setup.bash
ros2 topic hz /quanergy/points
```
It is also possible to run the node directly rather than using the launch file. This exposes more options but complicates running multiple instances simultaneously.
```
ros2 run quanergy_client_ros client_node --help
```

## Optional Configuration
To add ROS environment configuration automatically to every future bash session
```
echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
echo "source ~/QuanergySystems/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
```
