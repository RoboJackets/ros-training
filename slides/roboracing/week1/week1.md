---
title: Week 1
---
# Week 1

---

## What are we doing today?
- Introduction
- Review Installation
- RoboRacing Competitions
- Introduction to ROS

---

## Meet Daniel

- Daniel Martin
    - 2nd year, CS Major (Devices, Intelligence)
    - RoboRacing Software Lead
- How to contact me
    - Slack: [@Daniel Martin](https://robojackets.slack.com/messages/@Daniel_Martin)
    - Email: [dmartin1@gatech.edu](mailto:dmartin1@gatech.edu)

---

## Installation
- We work on Ubuntu 18.04 and ROS Melodic
- VM
    - VirtualBox or VMare
    - Team ROS Image [here](https://mega.nz/#!enoWBawC!_pOgFVwLxf4yGZyqckouGeabjJCD0xL2jwfA08lTS9w)
- Dual-booting
    - Be careful!
- ROS Installation Instruction
    - Wiki Guide [here](https://wiki.robojackets.org/RoboRacing_Software_Installation_Instructions)

---

## RoboRacing Competitions

-  Compete in two competitions!
-  International Autonomous Robot Racing Competition [(IARRC)]([https://iarrc.org)
-  evGrandPrix [(EVGP)](https://evgrandprix.org/autonomous/)
-  Both autonomous navigation competitions

<img src="https://i.imgur.com/0TSI6gt.jpg" height=300>

---

## What is ROS?
- Robotic Operating System
- Message passing framework for programming robots
- ROS is composed of three major parts: roscore, nodes, and topics

---

## How does ROS work?
- Small programs, called **nodes** communicating with each other
- Each **node** only responsible for one aspect
    - Makes the stack cleaner
- Example Nodes:
    - Identifying Barrels
    - Localization (Figuring out where we are)
    - Mapping
    - Path Planning

---

## Build the training repo
- Open a terminal window

```bash
cd catkin_ws
catkin_make
```

---

## Running ROS Nodes

---

### roscore
- roscore is the core system that controls ROS code and logical infrastructure
- Before we can run any nodes, need to run roscore:
    ```bash
    roscore
    ```
- Think of it as something that handles ROS functionality

---

### Launching `buzzsim`
- Let's launch the simulator:
    ```bash
    rosrun igvc_buzzsim buzzsim
    ```
<img src="https://i.imgur.com/jsRD8i5.png" height=500>

---

### `rosrun` in detail:
```bash
rosrun {package name} {executable name}
```

- ROS package is an organizational unit under which we can put ROS nodes, libraries, scripts and more
- For example, `rosrun igvc_buzzsim buzzsim`
    - We are running the `buzzsim` executable which lives in the `igvc_buzzsim` package

---

## ROS Topics & Messages
- Let's try and move the turtle around
- We need to communicate to the simulator node with ROS **Topics** and ROS **Messages**

---

## ROS Topic & Messages
- Think of a ROS **topic** as an an address that messages are sent to
- The ROS **message** is the actual message sent on that topic

<img class="stretch" src="https://www.mathworks.com/help/examples/ros/win64/ExchangeDataWithROSPublishersAndSubscribersExample_01.png">

---

## Example: Buzzsim
- Buzzsim node listens to messages on the `/oswin/velocity` topic to make the turtle move.

<img class="stretch" src="https://drive.google.com/uc?export=download&id=1eMJvdkT4IcLWoMIsSyF7P_bzkNNJW8yU">

---

## Making the turtle move in Buzzsim
- We can send messages to the `/oswin/velocity` topic to control the turtle with our keyboard!
- Use someone else's node `teleop_twist_keyboard` to do that:
    ```bash
    sudo apt-get install ros-melodic-teleop-twist-keyboard
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity
    ```
- We're launching the script `teleop_twist_keyboard.py` from the package `teleop_twist_keyboard`
- You should be able to control the turtle now!

---

## Making the turtle move in Buzzsim
- The `teleop_twist_keyboard` node **publishes** the velocity commands to the `/oswin/velocity` topic
- The `buzzsim` node **subscribes** to the `/oswin/velocity` topic and uses the received messages to move the turtle

<img class="stretch" src="https://drive.google.com/uc?export=download&id=1IMTA2zjoZJ-V2lv87u_clImAZ7xkhvEK">

---

## **Publishers** and  **Subscribers** in ROS
- `teleop_twist_keyboard` **publishes** to a topic, it's a ROS **Publisher**
- `buzzsim` **subscribes** to a topic, so it's a ROS **Subscriber**

ROS **Topics**, **Messages**, **Publishers** and **Subscribers** are the core parts of ROS

---

## `rostopic`
- To find out more information about ROS Topics, we can use `rostopic`:
```bash
rostopic
# rostopic is a command-line tool for printing information about ROS Topics.
# Commands:
# 	rostopic bw	display bandwidth used by topic
# 	rostopic delay	display delay of topic from timestamp in header
# 	rostopic echo	print messages to screen
# 	rostopic find	find topics by type
# 	rostopic hz	display publishing rate of topic    
# 	rostopic info	print information about active topic
# 	rostopic list	list active topics
# 	rostopic pub	publish data to topic
# 	rostopic type	print topic or field type
# 
# Type rostopic <command> -h for more detailed usage, e.g. 'rostopic echo -h'
```

---

### `rostopic list`
- Let's try out `rostopic list`
    ```bash
    rostopic list
    # /oswin/velocity
    # /rosout
    # /rosout_agg
    # /tf
    ```
- Notice that the `/oswin/velocity` topic is shown

---

### `rostopic info`
- We can get more information about the `/oswin/velocity` topic with `rostopic info`:
    ```bash
    rostopic info /oswin/velocity
    # Type: geometry_msgs/Twist
    #
    # Publishers:
    # * /teleop_twist_keyboard (http://robojackets:46001/)
    #
    # Subscribers: 
    #  * /buzzsimsim (http://robojackets:36899/)
    ```
- The *Type* of a topic is the type of message for that topic
- In this case, it is `geometry_msgs/Twist`

---

### Message types
- Since `geometry_msgs/Twist`, which comes from the package `geometry_msgs` package from ROS, we can
lookup the definition in the [ROS docs](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
(Google "geometry_msgs/Twist" and it should show up)

<img class="stretch" src="https://i.imgur.com/uovZmPe.png">

What about looking at what's getting published on a topic?

---

### `rostopic echo`
- We can use `rostopic echo` to look at the messages on a topic:

```bash
rostopic echo /oswin/velocity
# linear:
#   x: 0.5
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# ---
```

---

### `rostopic pub`
- What if we wanted to publish to a topic from the command line?
    - We can use `rostopic pub`
- Close `teleop_twist_keyboard`
- Publish using `rostopic pub`
    ```bash
    rostopic pub /oswin/velocity geometry_msgs/Twist "linear:
      x: 1.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"
    ```
- The turtle in the simulation should move
- Try playing around with the `x` component of `linear` and the `z` component of `angular`

---

## Summary
This week, we learnt about:
- [Running ROS Nodes](#/9)
    + `rosrun <package-name> <node-name>`, ie. `rosrun igvc_buzzsim buzzsim`
- [ROS Topics and `teleop_twist_keyboard`](#/10)
    + A ROS topic is like an address that messages are sent to
    + A ROS node can **subscribe** and **publish** messages to any topic
    + Each topic has a message type.
- [The `rostopic` tool](#/13)
    + `rostopic list` to list available topics
    + `rostopic echo` to listen to topics
    + `rostopic pub` to publish to topics
    
---

## Summary
The presentation slides are available on github, under the [ros-training](github.com/RoboJackets/ros-training)
repository on the RoboJackets github.

There's also a link to the
[markdown version of the same content](https://github.com/RoboJackets/ros-training/blob/master/code/instructions/week1.md)
in the README of the repository.

---

## Next Week
We'll be learning how to write ROS **Publishers** and **Subscribers** in C++

---

See you next week!
