# Introduction
Welcome to RoboJacket's ROS training exercises! These are a few exercises created in order to
teach the basics of ROS as well as a few simple robotics concepts.

## Prerequisites
This tutorial assumes that you have **ROS melodic** installed. If you haven't installed it, refer to
[ros-installation.md](ros-installation.md) for installation instructions. Alternatively, refer to
[these instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) for the complete set of installation instructions.

## Getting Started
First, install Git LFS. You can find Linux installation instructions [here](https://help.github.com/en/github/managing-large-files/installing-git-large-file-storage).  
```bash
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install
```   
  
Clone the repo into a catkin workspace:
```bash
cd catkin_ws # cd to where your catkin workspace is located
cd src
git clone https://github.com/RoboJackets/ros-training.git
```

Install dependencies:
```bash
sudo apt install libcgal-dev
cd catkin_ws # cd to where your catkin workspace is located
rosdep install --from-paths src --ignore-src -y 
```

Build the simulator and the exercises:
```bash
cd catkin_ws # cd to where your catkin workspace is located
catkin_make
```

And that's setup done. Head over to [week1](week1.md) to learn about ROS nodes.

