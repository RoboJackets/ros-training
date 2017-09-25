# What are we doing today?

-   Introductions
-   IGVC and RoboRacing Competitions
-   Setting up the IGVC/RoboRacing VM
-   Introduction to ROS


# ROS Training

<div class="NOTES">
talk about RoboJackets is one team and how training is a shared venture

</div>

-   IGVC and RoboRacing software use similiar tech stacks
    -   C++
    -   Robot Operating System
    -   OpenCV


# Developing for IGVC/RoboRacing

<div class="NOTES">
Downloading will take a while so make sure everyone does this now. Check for anyone with a dual boot already.

</div>

-   we use Ubuntu 16.04
-   we have a VM image that we have aready set up for you
    -   [Download Link](https://mega.nz/#!RugyRAQR!mFFuc_B3NEjq0DCbVzqIgQupDBGUpxFNpHOd9uNz1F0)
    -   We have flash drives if you have not already downloaded this
-   virtualbox is a virtualization software that robojackets supports
    -   [Virtual Box](https://www.virtualbox.org/wiki/Downloads)


# Ros Training Trainers

<div class="NOTES">
Be sure to mention joint training is called ROS training

</div>


# Meet Jason

![img](https://i.imgur.com/izC5WWA.jpg)

-   Jason Gibson
    -   Junior, Computer Science (Threads: Devices, Intelligence)
    -   Inside RoboJackets: Training Chair, IGVC Software Lead
    -   Outside RoboJackets: Avid lover of dad jokes
-   How to contact me
    -   Slack: [@jasongibson](https://robojackets.slack.com/messages/@jasongibson/)
    -   Email: [jgibson37@gatech.edu](mailto:jgibson37@gatech.edu)


# Meet Sahit

![img](https://i.imgur.com/aqKGrKm.jpg)

-   Sahit Chintalapudi
    -   Sophomore, Computer Science (Threads: Intelligence, Theory)
    -   Roboracing Software Lead & Roboracing PR manager
    -   S-tier Taylor Swift fanboy
-   How to contact me
    -   Slack: [@schintalapudi](https://robojackets.slack.com/messages/@schintalapudi/)
    -   Email: [schintalapudi@gatech.edu](mailto:schintalapudi@gatech.edu)


# Meet Evan

![img](https://i.imgur.com/50oKGjv.jpg)

-   Evan Bretl
    -   2nd year, Computer Science (Threads: Intelligence, Modeling/Simulation)
    -   Project Manager for RoboRacing
    -   Plays a lot of volleyball
-   Contact
    -   Slack: [@ebretl](https://robojackets.slack.com/messages/@ebretl/)
    -   Email: [evan.bretl@gatech.edu](mailto:evan.bretl@gatech.edu)


# Meet Matthew

![img](https://i.imgur.com/gMOFiLz.jpg)

-   Matthew Keezer
    -   1st year Masters, Computer Science (Threads: N/A)
    -   IGVC member, (retired IGVC software lead)
    -   583 games on steam, only 20% are unplayed <https://goo.gl/QzrMWZ>
-   Contact
    -   Slack: [@rmkeezer](https://robojackets.slack.com/messages/@rmkeezer/)
    -   Email: [rkeezer3@gatech.edu](mailto:rkeezer3@gatech.edu)


# IGVC

<div class="NOTES">
Talk a little bit about what the competition is like

</div>

-   Intelligent Ground Vehicle Competition
-   an autonomous navigation competition
-   [Competition Website](http://www.igvc.org)

![img](https://i.imgur.com/40QJ9g9.jpg)


# Sensors

<div class="NOTES">
briefly mention what each sensor is

</div>

-   GPS
-   lidar
-   camera
-   IMU
-   encoders


# IGVC Software

<div class="NOTES">
mention ROS is an industrial standard

</div>

-   c++ based
-   our software can be broken down into three parts
    -   path planning
    -   obstacle detection
        -   lidar
        -   computer vision
    -   mapping and localization
-   all the details will be covered in later weeks


# RoboRacing

-   International Autonomous Robot Racing Competition (Waterloo, Canada)
-   Autonomous Vehicle Competition (Denver, Colorado)
-   Both are autonomous racing competitions
-   Same sensors as IGVC but **no GPS**

![img](https://i.imgur.com/8s99LgL.jpg)


# RoboRacing Software

-   Consists of a few packages, but at a high level we have: vision, planning, and control.
-   the IARRC package and AVC packages contain competition specific logic (perception)
-   common has our path planner which is common between both events
-   platform consists of controls code that interfaces with the arduino
-   gazebo has the software that supports simulation


# ROS

-   message passing interface between multiple concurrent threads
-   multitude of useful libraries


# ROS Architecture

<div class="NOTES">
the arrows are topics and the circles are nodes

</div>

-   nodes
    -   small programs that are all running at the same time
-   topics
    -   messages that are passed between different nodes

![img](https://magiccvs.byu.edu/wiki/images/b/bb/Rqt_graph_turtle_key.png)


# Fork Our Repo

-   RoboJackets/igvc-software (<https://github.com/RoboJackets/igvc-software>)
-   RoboJackets/roboracing-software (<https://github.com/RoboJackets/roboracing-software>)

![img](https://i.imgur.com/9Wz6RP3.png)


### Ensure you have Virtualization turned on in your BIOS

-   [This](http://www.howtogeek.com/213795/how-to-enable-intel-vt-x-in-your-computers-bios-or-uefi-firmware/) is a simple guide of how to do this.
-   While this is not 100% necessary, it will make your VM much faster.
-   On a Windows host, you may need to turn off Hyper-V as well.


### 1. Go to `File->Import Appliance`

![img](https://i.imgur.com/keQmMy4.png)


### 2. Select the `.box` file you extracted earlier

![img](https://i.imgur.com/3S2Pgt3.png)


### 3. Increase the Amount of Memory and CPU's

-   Increase the Memory/CPU to your computer's specs. Don't allocate too much memory/cpus!

![img](https://i.imgur.com/P8Adm2a.png)


### 4. Hit `Import`!


### Configure Settings of Imported Image


### 1. Right click your new virtualbox entry, and hit `settings`


### 2. Increase the Amount of Video RAM, and turn on 3D Acceleration

-   If you do not have virtualization, virtualbox may not allow you to turn on 3D Acceleration

![img](https://i.imgur.com/YzmNmcM.png)


### 3. Turn **OFF** `Remote Display`

![img](https://i.imgur.com/cvigW2G.png)


### Boot your new VM

-   Double Click the Entry, or Right Click -> Start -> Normal Start


# Fix remotes

-   open a terminal

```shell
cd catkin_ws/src/ros-training
git remote rename origin upstream
git remote add origin https://github.com/<USERNAME>/ros-training.git
git pull upstream
```


# Build Our Code

-   open a terminal

```shell
cd catkin_ws
catkin_make
```


# rviz

<div class="NOTES">
show them how to visualize things in rviz. Explain major topics. Use Gazebo. Both teams. Show rqt<sub>graph</sub> have them do it also.

</div>

-   a visualization tool that works well for ROS
    -   we are seeing messages our robot is sending


# Bag Files

-   A bag file is a way of recording all the messages written to topics
    -   more on that next week, for now think of them as a log file
-   We can visualize the information provided by these bag files to help test new code


# Run Our bag IGVC

<div class="NOTES">
tell them about tab complete

</div>

-   open terminator

```shell
roscore
```

-   split terminator

```shell
cd Desktop
rosbag play -l IGVC_sim_bag.bag
```

-   split terminator

```shell
rviz
```


# Run Our Code IGVC

<div class="NOTES">
show them on your machine

</div>

-   split terminator

```shell
roslaunch igvc mapper.launch
```

-   split terminator

```shell
roslaunch igvc pather.launch
```


# Run Our bag RR

<div class="NOTES">
tell them about tab complete

</div>

-   open terminator

```shell
roscore
```

-   split terminator

```shell
cd Desktop
rosbag play -l RR_bag.bag
```

-   split terminator

```shell
rviz
```


# Run Our Code RR

<div class="NOTES">
show them on your machine

</div>

-   split terminator

```shell
roslaunch avc avc.launch
```