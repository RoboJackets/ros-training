# What are we doing today

-   IMU
-   Gazebo
-   Review what makes up a node
-   Write a node to control a simulated robot.


# IMU

-   Inertial measurement unit
-   Detects linear acceleration using accelerometers
-   Detects rotational rate using gyroscopes
-   Can include magnetometer for headings
-   Super useful for position estimates
-   Suffers from accumulated error


## RPY

-   Roll, Pitch, Yaw

![img](https://upload.wikimedia.org/wikipedia/commons/thumb/5/54/Flight_dynamics_with_text.png/1600px-Flight_dynamics_with_text.png)


# Gazebo

-   The simulator used by both RoboRacing and IGVC

![img](https://upload.wikimedia.org/wikipedia/en/thumb/1/13/Gazebo_logo.svg/1024px-Gazebo_logo.svg.png)


# Simulation

<div class="NOTES">
talk about how it crashes. Also that we can simulate friction, mass, etc

</div>

-   simulators are wonderful
-   allow to test code in somewhat real life situations
-   not a ROS product
    -   interacts with ROS through plugins that publish to topics
    -   these plugins are not perfect so it crashes **a lot**


# Package Structure

-   `TEAM_NAME_Description`
    -   contains the URDF file
    -   defines the world
-   `TEAM_NAME_Control`
    -   contains the nodes that control the environment


# URDF

-   unified robot description format
-   this is what gazebo uses to generate the robot
-   XML file format
-   defines everything in a 3-dimensional grid


# Links

<div class="NOTES">
take about what each is and how meshes can be used as geometries. visual is required

</div>

-   links contain
    -   required for ROS
        -   visual geometry
    -   required for gazebo
        -   collision geometry
        -   intertial geometry


# Links Basic Example

<div class="NOTES">
make sure to launch rviz with this urdf

</div>

```XML
<robot>
  <link name="base_link">
    <visual>
      <geometry>
	<box size="0.8 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```


# Gazebo Link

<div class="NOTES">
launch this in gazebo

</div>

```XML
<link name="body">
  <inertial>
    <origin xyz="0 0 0" />
    <mass value="50.0" />
    <inertia  ixx="0.0" ixy="0.0"  ixz="1.0"  iyy="0.0"  iyz="0.0"  izz="0.0" />
  </inertial>
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="model://urdf/meshes/Body.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="model://urdf/meshes/Body.dae"/>
    </geometry>
  </collision>
</link>
```


# Joints

-   links can be connected using joints
    -   all joints have a parents and a child
-   all positions are realtive to its parents
    -   entire tree should have a single root


# I AM ROOT

![img](https://www.syfy.com/sites/syfy/files/wire/legacy/groot_0.jpg)


# Joint Types

| type         | usage                                      |
|------------ |------------------------------------------ |
| `continuous` | rotates in an axis and has to limits       |
| `fixed`      | does not move                              |
| `floating`   | 6 degrees of freedom                       |
| `revolute`   | rotates on an axis and has rotation limits |


# Example Joint

```XML
<joint name="base_link_to_left_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <axis xyz="0 0 -1" rpy="0 0 0" />
  <origin xyz="0 0 0" rpy="0 0 0" />
</joint>
```


# Useful Commands

<div class="NOTES">
explain the output

</div>

-   `check_urdf`
    -   "compiles" the urdf and gives a printout of the tf tree

```BASH
robot name is: hal
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 4 child(ren)
  child(1):  base_link
  child(2):  body
    child(1):  back_ball
    child(2):  left_wheel
    child(3):  right_wheel
  child(3):  usb_cam_center
    child(1):  optical_cam_center
  child(4):  lidar
```


# Introducing HAL

![img](https://i.imgur.com/IGlRSWv.png)


# Review


## What are the three files every node needs?


### What are the three files every node needs?

-   CMakeLists.txt
-   package.xml
-   a source file


## What is in a CMakeLists.txt file?


### What is in a CMakeLists.txt file?

-   How to build the node
    -   Defines the structure of the package for the compile
    -   Linking the executable ROS nodes to the appropriate libraries (OpenCV, PCL, etc..)


## What is in the package.xml file?


### What is in the package.xml file?

-   What are the dependencies of a node
-   maintainer information, author, etc


## What is required for a subscriber?


### What is required for a subscriber?

-   Callback function
-   Topic name
-   Queue size


## What is required for a publisher?


### What is required for a publisher?

-   Topic name
-   Queue size


## What things must be done in every node?


### What things must be done in every node?

-   include ros header
-   ros::init
-   Create NodeHandle
-   ros spin


## How do I get what nodes are currently running?


### How do I get what nodes are currently running?

-   rosnode list


## How do I get information about a running node?


### How do I get information about a running node?

-   rosnode info [NAME]


## How do I get a list of the topic currently publishing?


### How do I get a list of the topic currently publishing?

-   rostopic list


## How do I get how often a topic is publishing?


### How do I get how often a topic is publishing?

-   rostopic hz [NAME]


## How do I get what is being published on a topic?


### How do I get what is being published on a topic?

-   rostopic echo [NAME]


# Output

-   Must have output set to screen to print out to the terminal
    -   Done in the launch file


## `ROS_INFO`

-   `ROS_INFO("Hello");`
-   Prints out whatever is inside
-   Cannot print out multiple things


## `ROS_INFO_STREAM`

-   `ROS_INFO_STREAM("x = " << point.x << " y = " << point.y);`
-   You can pipe information just like with cout


# PCL

-   The Point Cloud Library
-   Standard across most ROS projects
-   Lots of useful data structures and algorithms focused on points


# What is a point cloud?

-   A collection of points
    -   Typically XYZ


# Write a node

-   Now we want to have hal set himself facing an object