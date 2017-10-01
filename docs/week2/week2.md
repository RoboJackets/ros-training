# Meet the lidar!

-   This sensor shines a laser light in circles and measures the return times from the pulses to give us a 3D model of our world
-   Notice that the lidar has a **rotating** component. This makes them more fragile the a sensor like a camera, so **they should be handled with caution**.

![img](https://cdn.sparkfun.com//assets/parts/1/2/0/0/3/14117-02a.jpg)


# Pointclouds

-   We use ROS's PCL package to interface with the LIDAR, and get messages of type [PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
-   A pointcloud is a data structure for holding the results found by the lidar (X, Y, Z)
-   This includes an array of points picked up from the the LIDAR and their cartesian coordinates relative to the LIDAR


# What does a pointcloud look like?

<div class="NOTES">
Pull out Jason's LIDAR and run rqt

</div>

-   Live demo time!


# How do we structure ROS projects?

<div class="NOTES">
pull open the repository and show them this literal structure

</div>

```bash
project
  package1
      src
	node1a
	node1b
	...
      CMakeLists.txt
      package.xml
  package2
      src
	node2a
	node2b
	...
      CMakeLists.txt
      package.xml
  ...
```


# CMakeLists file

<div class="NOTES">
walk them through the file and show them the parts of it

</div>

-   ROS code is built with a build tool called catkin, which itself sits on top of a build tool called CMake
-   CMakeLists defines the structure of the package for the compiler, as well as linking the executable ROS nodes to the appropriate libraries (OpenCV, PCL, etc..)
-   [Example CMakeLists](https://github.com/RoboJackets/roboracing-software/blob/master/iarrc/CMakeLists.txt)


# Our CMakeLists.txt file

```
cmake_minimum_required(VERSION 2.8.3)
project(node_example)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```


# package.xml file

<div class="NOTES">
walk them through the file and show them the parts of it

</div>

-   Defines packages the host computer needs to have installed to run the code in the project
-   `build_depends` are packages needed to compile the code
-   `run_depends` are packages needed by the code at runtime
-   Also specifies project maintainer/contact information
-   [Example package.xml](https://github.com/RoboJackets/igvc-software/blob/master/gazebo/igvc_control/package.xml)


# Our package.xml file

```xml
<?xml version="1.0"?>
<package format="2">
  <name>node_example</name>
  <version>0.0.1</version>
  <description>
  basic publisher and subscriber for IGVC and RR ROS training
  </description>
  <author>Jason Gibson</author>
  <maintainer email="jgibson37@gatech.edu">Jason Gibson</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>

  <depend>roscpp</depend>
  <depend>std_msgs</depend>

</package>

```


# Some ROS Vocab

-   A <span class="underline">node</span> is a process running on your computer
    -   you specify the behavior of the node with the C++ code you write
-   Nodes <span class="underline">publish messages</span> on a <span class="underline">topic</span>
    
    -   A message is a data structure made up of fields. Messages can be
    
    primitive like booleans or ints, or they can contain other data structures like a PointCloud
    
    -   Think of a topic as the mailbox to which the messages get delivered
-   Nodes <span class="underline">subscribe</span> to a topic to receive <span class="underline">callbacks</span> when a new message appears
    
    -   Subscribing to a topic entails telling ROS to call a function you
    
    define everytime a new message is published on this topic!


# A ROS Publisher - Setup

<div class="NOTES">
Should we write our own? I like how well this is documented

</div>

-   Borrowed from: <https://raw.githubusercontent.com/ros/ros_tutorials/kinetic-devel/roscpp_tutorials/talker/talker.cpp>

```C++
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "talker");

   ros::NodeHandle n;

   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

   ros::Rate loop_rate(10);
```


# ROS Publisher - publishing

```C++
  int count = 0;
  while (ros::ok())
  {
  //This is a message object. You stuff it with data, and then publish it.
  std_msgs::String msg;

  std::stringstream ss;
  ss << "hello world " << count;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  chatter_pub.publish(msg);

  ros::spinOnce();

  loop_rate.sleep();
  ++count;
  }
  return 0;
}
```


# Start the publisher node

```sh
rosrun node_example talker
```


# rosnode

<div class="NOTES">
demo

</div>

| `rosnode list` | lists all of the nodes that are running                       |
| `rosnode info` | gives information about a node (publishers, subscribers, etc) |
| `rosnode kill` | kills a running node                                          |


# rostopic

<div class="NOTES">
demo

</div>

| `rostopic list` | lists the currently publishing or subscribing topics    |
| `rostopic info` | prints out information about that topic                 |
| `rostopic echo` | prints out the messages on that topic                   |
| `rostopic hz`   | prints out the rate at which a topic is being published |
| `rostopic type` | prints out the type of message                          |
| `rostopic find` | prints out the topics with the given message type       |
| `rostopic pub`  | publishes a given topic with the given command          |


# rosmsg

<div class="NOTES">
demo

</div>

|              |                                   |
|------------- |---------------------------------- |
| `rosmsg show` | shows you the make up of a message |

-   `rostopic type topicName | rosmsg show`


# A ROS Subscriber -  Setup

```C++
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```


# A ROS Subscriber - Subscribing

```C++
int main(int argc, char **argv)
{
   ros::init(argc, argv, "listener");

   ros::NodeHandle n;

   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

   ros::spin();

   return 0;
}
```


# Start the subscriber node

```sh
rosrun node_example listener
```


# Let's go back to PointCloud2

<div class="NOTES">
Talk them through the layout of the message. Show them that messages can consists of field which then consist of other fields. For example, click on the header, which links to message definition for a header. Note that every message should have a header

</div>

-   Everyone visit the [PointCloud2 message declaration](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)


# Takeaways from the message definition

-   Messages should always have headers. Haveing a timestamp makes logging and debugging easier
-   Messages contain fields of data, some of these fields also contain fields of data, as they are also message types


# `rqt_graph`

<div class="NOTES">
make sure to launch enough nodes to make it hella complicated. talk a little bit about a code base

</div>

-   what if I want to see all of my nodes and messages
-   `rqt_graph`