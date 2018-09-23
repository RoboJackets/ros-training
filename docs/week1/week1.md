# What are we doing today?

-   Introductions
-   IGVC Competition
-   Introduction to ROS
-   Setting up the IGVC VM


# Initial Instructions

-   We have a ~3 GB file for you to download.
-   Here is a link: <https://mega.nz/#!rl80jQoY!_8Mqr_LClq3mIExEe2OztWqoSRrkbAga0aeqHkv2dSg>
    -   We have it on flash drives for easy installation
-   This is a VM image. In leiu of dual-booting, this is technically a method of running linux.


# ROS Training

<div class="NOTES">
We use a lot of technologies, do not worry about it so much

</div>

-   IGVC's tech stack is built with a few important technologies
    -   C++
    -   Robot Operating System
    -   Tensorflow


# Ros Training Trainers


## Meet Andrew

<>

-   Andrew Tuttle
    -   3rd Year, Computer Science (Threads: Theory, Intelligence)
    -   Inside RoboJackets: IGVC Software Lead
    -   Outside RoboJackets: Dungeon Master and general geek
-   How to contact me
    -   Slack: [@Andrew Tuttle](https://robojackets.slack.com/messages/D74EXN804/)
    -   Email: [atuttle7@gatech.edu](mailto:atuttle7@gatech.edu)


## Meet Jason

![img](https://i.imgur.com/izC5WWA.jpg)

-   Jason Gibson
    -   Senior, Computer Science (Threads: Devices, Intelligence)
    -   Inside RoboJackets: President
    -   Outside RoboJackets: Avid lover of dad jokes
-   How to contact me
    -   Slack: [@Jason Gibson](https://robojackets.slack.com/messages/@Jason_Gibson/)
    -   Email: [jgibson37@gatech.edu](mailto:jgibson37@gatech.edu)


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
    -   path planning and following
    -   obstacle detection
        -   lidar
        -   computer vision
    -   mapping and localization
-   all the details will be covered in later weeks

![img](https://imgur.com/N8fUcwt.jpg)


# ROS

-   message passing interface between multiple concurrent threads
-   We use ROS to enable information flow to and from our computer, sensors, and motors.


# ROS Architecture

<div class="NOTES">
topics->channels messages->videos

</div>

-   ROS is composed of three major parts: ROSCORE, Nodes, and Topics.


# ROSCORE

-   ROSCORE is the core system that controls ROS code and logical infrastructure. - ROSCORE manages the information flow throughout our program, making sure messages are sent and recieved by the proper sources and sinks, called nodes.


# Nodes

-   ROS nodes are programs that do all the processing work in IGVC.
-   They collect information from sensors and perform some processing upon that information, and may or may not produce information themselves.
-   They are managed by ROSCORE, which determines what information goes to which nodes through the third component, topics.


# Topics

-   Topics are the classifications that ROS sorts messages by.
-   Nodes can send messages to particular topics though a process called publishing and conversely request information from a topic through a process called subscribing.
-   ROSCORE and nodes use topics as a method of information routing, allowing nodes to retrieve specific information from ROSCORE.


# YouTube ~ ROS?

-   All this talk of publishing and subscribing reminds me of YouTube.

![img](https://imgur.com/XxZ5vQx.jpg)


# YouTube ~ ROS?

-   If videos are messages, then YouTube is like ROSCORE.
-   Nodes are like YouTube users, and topics are like channels.
-   Users can create videos (messages) and ask YouTube (ROSCORE) to publish those messages to a particular channel (topics)
-   Users can also ask YouTube for videos of a particular type by providing a channel name.


# ROS talker and listener

-   Let's do a quick walkthrough of a basic node
    -   We will cover this more in detail later, ignore the syntax and see what is happening
-   


## Some ROS Vocab

-   A <span class="underline">node</span> is a process running on your computer
    -   you specify the behavior of the node with the C++ code you write
-   Nodes <span class="underline">publish messages</span> on a <span class="underline">topic</span>
    
    -   A message is a data structure made up of fields. Messages can be
    
    primitive like booleans or ints, or they can contain other data structures like a PointCloud
    
    -   Think of a topic as the mailbox to which the messages get delivered
-   Nodes <span class="underline">subscribe</span> to a topic to receive <span class="underline">callbacks</span> when a new message appears
    
    -   Subscribing to a topic entails telling ROS to call a function you
    
    define everytime a new message is published on this topic!


## A ROS Publisher

<div class="NOTES">
Should we write our own? I like how well this is documented

</div>

-   [Example Code](https://github.com/RoboJackets/ros-training/blob/master/code/week1/src/talker.cpp)


## Subscriber

-   [Example Code](https://github.com/RoboJackets/ros-training/blob/master/code/week1/src/listener.cpp)


## Demo!!


# Developing for IGVC

<div class="NOTES">
Downloading will take a while so make sure everyone does this now. Check for anyone with a dual boot already.

</div>

-   we use Ubuntu 18.04.1
-   we have a VM image that we have aready set up for you
    -   [Download Link](https://mega.nz/#!rl80jQoY!_8Mqr_LClq3mIExEe2OztWqoSRrkbAga0aeqHkv2dSg)
    -   We have flash drives if you have not already downloaded this
-   virtualbox is a virtualization software that robojackets supports
    -   [Virtual Box](https://www.virtualbox.org/wiki/Downloads)


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


# ~Fix remotes

-   open a terminal

```shell
cd catkin_ws/src/ros-training
git remote rename origin upstream
git remote add origin https://github.com/<USERNAME>/ros-training.git
git pull upstream
```


# Fork Our Repo

-   RoboJackets/igvc-software (<https://github.com/RoboJackets/igvc-software>)

![img](https://i.imgur.com/9Wz6RP3.png)


# Build Our Code

-   open a terminal

```shell
cd catkin_ws
catkin_make
```