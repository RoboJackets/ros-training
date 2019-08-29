# Installing ROS
This installation guide was largely copied from [wiki.ros.org](http://wiki.ros.org/melodic/Installation/Ubuntu)
will assume that you are on Ubuntu 17 or 18.

## Configure your Ubuntu repositories
Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the
[Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this. 

## Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Set up your keys 
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

## Installation
First, make sure your Debian package index is up-to-date:
```bash
sudo apt update
```

Then install the Desktop-Full Install:
```bash
sudo apt install ros-melodic-desktop-full
```

## Initialize rosdep
Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies
for source you want to compile and is required to run some core components in ROS. 

## Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time
a new shell is launched: 
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Next steps
Now that we've installed ROS, it's time to [setup the catkin workspace](workspace-setup.md).
