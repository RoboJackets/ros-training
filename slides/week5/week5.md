---
title: Week 5
revealOptions:
    width: 1440
    height: 1080
---
# Week 5

---
## Recap of last week

----

#### IMU 

##### What is an IMU?

Inertial Measurement Unit <!-- .element: class="fragment" data-fragment-index="0" --> 
Measures acceleration, angular velocity, and heading <!-- .element: class="fragment" data-fragment-index="1" --> 

----

#### Coordinate Frames

##### What is a coordinate frame?

Defines a set of directions relative to some object <!-- .element: class="fragment" data-fragment-index="0" --> 

----

#### Localization

##### What is localization?

Finding where you are <!-- .element: class="fragment" data-fragment-index="0" --> 

----

##### What is one way to do localization?

Dead Reckoning <!-- .element: class="fragment" data-fragment-index="0" --> 

----

#### Dead Reckoning

##### How does dead reckoning work?

Integrate accelerations and velocities to obtain position with kinematics <!-- .element: class="fragment" data-fragment-index="0" --> 

----

#### Visualization

##### How can we visualize things in ROS? (ie. Localization node)

rviz <!-- .element: class="fragment" data-fragment-index="0" --> 

---

## Welcome to Week 5

- LiDAR
- Occupancy Grid Maps

---

## Lidar
- **Li**ght **D**etection **a**nd **R**anging
- Method of measuring distances to a target
    - Shoot a _laser_ and measuring the reflected light
    - Distance obtained by **time of flight** 
    
----

- Take distance measurements around the robot
- Allow the robot to "see" obstacles
    - _2D_ or _3D_
    
----

<img style="max-width: 1000px; max-height: 1000px" src="https://i.imgur.com/qtBu0uC.png" />

----

<img style="max-width: 1000px; max-height: 1000px" src="https://i.imgur.com/KAxJb0T.png" />

Notice that we are able to "see" the surrounding obstacles from the "shape" of the lidar scan. We can use this
to construct a map of our surroundings.

---

## Playing with lidar in the simualation with rviz and keyboard teleop
- Let's get a feel for what lidar data is like
- Launch `week5.launch`:
```bash
roslaunch week5.launch
```

----

The simulation should pop up with a lidar visualization like the above image

Open up rviz:
```bash
rviz
```

----

- Change the **fixed frame** in the top left corner of the left menu to `oswin` 
- Set the **target frame** in the right menu to `oswin` as well.

- Add the "PointCloud2" visualization so you can visualize the lidar data
    - Click **add** in the left menu, select "PointCloud2"
    - Change the "Topic" to `/oswin/pointcloud`.

----

In a third window, open up `teleop_twist_keyboard` as you have done many times already:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity
```

----

## Grid Maps

- How do you do mapping?
- One way is through the use of **grid maps**

----

- **grid map** uses a **grid** to represent the world
- The locations of any obstacles are then **discretized** into the grid cells.

----

Example:
- Dimension: 5m x 10m
- Resolution: of 10cm per grid cell

How many total grid cells?

(5m * 10 cells/m) * (10m * 10 cells/m) = 5000 <!-- .element: class="fragment" data-fragment-index="1" --> 

----

Ex:
- (0, 0) in the map corresponds to (0m, 0m) in the real world
- Obstacle (let's assume it's a single point) located at coordinates (2m, 7m).

What grid coordinates would this translate to?

(2m * 10 cells/m, 7 * 10 cells/m) = (20, 70) on the grid <!-- .element: class="fragment" data-fragment-index="1" --> 

----

Suppose there's another obstacle (also a single point) located at coordinates (2.01m, 7.01m).
What grid coordinates would that translate to?

(2.01m * 10 cells/m, 7.01 * 10 cells/m) = (20, 70) on the grid <!-- .element: class="fragment" data-fragment-index="1" --> 

----

- This is one of the disadvantages of using grid maps
- **Discretization error**, depending on the resolution of the map.

----

- Advantage: **easy** to work with in terms of **mapping** as well as **path planning**
    - We will get into this next week

---

## Coordinate Frames (_again_) with mapping and lidar
Before we start implementing a grid map, we need to look at coordinate frames again.

----

- Data from the lidar is obtained _relative to the sensor position_
- We want this data in the **global** coordinate frame, one of which is the "odom" frame
    - [REP 105](https://www.ros.org/reps/rep-0105.html)
    
We need to transform these coordinates from the sensor's _coordinate frame_ to the "odom" frame

----

- Assume our robot is at the coordinates (1,2) facing 90 degrees
    - x points East and y points North
    - with 0 degrees yaw being East and increasing CCW
- Lidar at same location sees an object at coordinates (3, -1) in the **robot**'s frame
    - Body frame's x points forward and y points left
- What are the coordinates of (3, -1) in the **odom** frame?

----

<img style="max-width: 1000px; max-height: 1000px" src="https://i.imgur.com/CRpqhMh.png" />

----


- (3, -1) in the odom frame has the coordinates of (2, 5) (Red is the x-axis, Green is the y-axis).
- Once we have transformed this lidar point into the **odom** frame, we can then insert this point into our map
    - Mark it as occupied
    
----
    
Mapping: repeat this procedure for every point in the lidar scan every time we get a lidar scan.

---

## Obtaining the coordinate transform and transforming pointclouds in C++
- Now that we know why, let's find out how

----

- Multiple ways of obtaining the transforms
    - Look up using **tf**, a useful package that makes coordinate transforms easy in ROS.

----
 
### Coordinate transforms with `tf::TransformListener`

- Need to use something called a `tf::TransformListener`
    - "listens" for coordinate transforms between different reference frames
    - listen for transforms broadcasted by `tf::TransformBroadcaster` in other nodes
    
We don't need to worry about the `tf::TransformBroadcaster` part as the simulation already takes care of that for us

----

To use a `tf::TransformListener`, first create one like so:
```c++
tf::TransformListener listener;
```

----

Then, to use it to lookup transforms, say from the frame "oswin" to the frame "odom", do the following:
```c++
tf::StampedTransform transform;
try {
  listener.lookupTransform("odom", "oswin", msg.header.stamp, transform);
} catch (tf::TransformException &ex) {
  ROS_ERROR_STREAM(ex.what());
}
```

----

```c++
lookupTransform (const std::string &target_frame,
                 const std::string &source_frame,
                 const ros::Time &time,
                 StampedTransform &transform) const 
```

- Pass in `msg.header.stamp` to specify that we want the transform at that moment
- Can also pass `ros::Time(0)` to signify we want the latest transform.

----

- Notice that we use a `try { ... } catch () { ... }` statement here
- This is because `listener.lookupTransform()` might **throw** an **exception** if some error occurs
    - Unable to find the transform requested
- If we don't add a `try { ... } catch() { .. }` here, then the program will crash when an exception is thrown

----

Common snippet:
```c++
tf::StampedTransform transform;
ros::Time stamp = msg.header.stamp;
if (!listener.canTransform("odom", "oswin", msg.header.stamp)
{
    stamp = ros::Time(0);
}
try {
  listener.lookupTransform("odom", "oswin", msg.header.stamp, transform);
} catch (tf::TransformException &ex) {
  ROS_ERROR_STREAM(ex.what());
}
```

---

### Transforming Pointclouds with `tf::Transform`
- Now it's time to actually transform a Pointcloud with this `tf::Transform`.
- First off though, what type is a Pointcloud?

----

- The details are kind of confusing for ROS
    - There have been a few different versions
    -  We'll use the type **`pcl::PointCloud<pcl::PointXYZ>`**
    
----

- Notice that `pcl::PointCloud` is a template as it uses angle brackets
- Don't worry about the template part of this though
    - Basically only use `pcl::PointXYZ`.

----

To transform a pointcloud, we can make use of a function from `pcl_ros`:
```c++
pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
pcl_ros::transformPointCloud(input_cloud, transformed_cloud, transform);
```

- `input_cloud` is the pointcloud you want to transform
- transform is the `tf::Transform` representing the transformation

---

### The actual map data type
- Before we forget, we need to store the map information somewhere
- One data type that can be easily visualized in
rviz (visualization is v. important) is `nav_msgs::OccupancyGrid`.
- Reminder: google it to find information about its fields

----

Some important points:
* The `info` member contains information about the map: width, height etc. 
* The `data` member (of type `std::vector<int8_t>` needs to be initialized
    * In `main` before you call `ros::spin()` make
sure that the `data` member is initialized to have as many elements as there are grid cells, ie.
```c++
map.data = std::vector<int8_t>(total_cells, 0);
```
* The map is 2D, but the `data` member is only 1D. This means that you'll need to convert from 2D coordinates to an
index.
  * This works by imagining an index that increments left to right, top to bottom. For example, for a 3x3 grid:
 
     0 1 2
     
     3 4 5
     
     6 7 8

---

## Implementing a simple mapping in ROS with C++
- You know everything you need to implement a simple mapper for lidar points in ROS
- Write your node in [src/week5/main.cpp](../igvc_training_exercises/src/week5/main.cpp).

- For this mapper, we'll increment the occupied grid cell by 1 every time
    - Until it reaches 100 (Since thats the maximum value)

----

1. Subscribe to `/oswin/pointcloud` and write a callback function taking in `pcl::PointCloud<pcl::PointXYZ>`
2. Create a global `tf::TransformListener` and call `lookupTransform` from `oswin` to `odom` inside the callback
3. Transform the pointcloud from the `oswin` frame to the `odom` frame using `pcl_ros::transformPointCloud`
4. Iterate through the pointcloud. For each point, find which grid cell it is, then increment that cell by 1 if its
less than 100
5. Create a publisher of type `nav_msgs::OccupancyGrid` and publish the map

----

- Open up rviz and add the `nav_msgs::OccupancyGrid` visualization
- See a map that gets populated with obstacles as you drive around in the simulation with

<img style="max-height: 1000px; max-width: 1000px" src="https://i.imgur.com/9JbAbHT.png" />

---

## Extensions:
- Occupancy grid mapping
    - Finding out wtf the "occupancy" part means
    - We really only covered the "grid mapping" part
    
---

Good luck!

---

## Summary

----

- [The LiDAR](#lidar)
    + Takes distance measurements around the robot
    + Allows the robot to "see" obstacles

----

- [Grid Maps](#grid-maps)
    + A grid map discretizes the world into cells
    + Easy to implement and use, but may have discretization errors

----

- [Coordinate Frames and Transforms](#coordinate-frames-_again_-with-mapping-and-lidar)
    + The coordinate of objects in different coordinate frames is different
    + We can use transforms to transform the coordinates of an object from one frame to another

----

- [Transforming Pointclouds in ROS](#transforming-pointclouds-with-tftransform)
    + Use the `pcl_ros::transformPointCloud` function to transform pointclouds to a different coordinate frame

----

- [Implementing a simple mapper](#implementing-a-simple-mapping-in-ros-with-c)
    + Getting practice with working with pointclouds and performing coordinate transforms
    
---

## Next Week
Next week, we'll look at performing **navigation** on the map that we've created this week

---

See you next week!
