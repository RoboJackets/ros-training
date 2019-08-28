# Introduction
Welcome to RoboJacket's ROS training exercises! These are a few exercises created in order to
teach the basics of ROS as well as a few simple robotics concepts.

## Getting Started
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

