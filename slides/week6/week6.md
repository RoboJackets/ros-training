---
title: Week 6
revealOptions:
    width: 1440
    height: 1080
---
# Week 6

---
## Recap of last week

----

#### Lidar

##### What is Lidar?
Light Detection And Ranging<!-- .element: class="fragment" data-fragment-index="0" --> 
Measure distances to obstacles around the robot<!-- .element: class="fragment" data-fragment-index="1" --> 

----

#### Grid Map
##### What is a Grid Map?
A way of representing the world by discretizing coordinates into grid cells.<!-- .element: class="fragment" data-fragment-index="0" --> 

----

#### Coordinate Frames
##### Why do we need them when mapping with a LiDAR?
The LiDAR receives data in the sensor's frame. Need to transform to world (odom) frame in order to find the grid cell.<!-- .element: class="fragment" data-fragment-index="0" --> 

----

##### How do we find coordinate transforms with tf?
<pre class="inline">tf::TransformListener</pre><!-- .element: class="fragment" data-fragment-index="0" --> 
<pre class="inline">listener.lookupTransform</pre><!-- .element: class="fragment" data-fragment-index="1" --> 

----

## Welcome to Week 6

- Path planning

---

## Path Planning

- Plan from **start** to **goal**
- Avoid obstacles in the world

----

### Paradigms of Path Planning

1. Grid-based search
2. Sampling based
3. Optimization based

----

### Grid-based Search

- Traditional approach to path planning
- Represent the map as a grid, search through the grid for a path
- Examples:
    - A*
    - D*

<img style="max-width: 500px; max-height: 500px" src="https://seangwyn.files.wordpress.com/2013/11/a-star-example.png" />

----

## Sampling based

- Randomly sampling in **configuration space**
    - Space of possible positions attainable by the robot
- Connect previous sampled points to new sampled points
- Repeat until gol is found
- Examples:
    - RRT
    - RRT*

<img src="https://www.researchgate.net/profile/Ahmad_Abbadi/publication/216452206/figure/fig2/AS:669443164233739@1536619096656/Example-of-RRT-Connect-with-the-trap-obstacle.png" />

----

## Optimization based
- Treat as optimization problem
- Have set of states **x**, a set of control actions **u** and some cost function **f(x, u)**\
- A path found by finding **x** and **u** which minimize the cost function
- Examples:
    - TEB Local Planner
    - RoboRacing Simulated Annealing planner

---

## Implementing a simple optimization based path planner

Let's implement a very simple optimization based path planner!

----

To do path planning though, we need a **map**
- We made a node that creates a map last week
- We'll be making use of that

----

### Simple optimization based path planner
- Look at 9 paths that have different angular velocities
    - -4 to 4
- Calculate the resulting pose after driving with that angular velocity for some time **Δt** (a parameter)
- Calculate the cost of each resulting pose
    - For now, current distance from `oswin` to `kyle`
    - But infinite cost if we hit an obstacle
- Pick the angular velocity that resulted in the lowest cost
- Publish the `geometry_msgs::Twist` message to `oswin/velocity`

----

### Implementation Plan
1. Subscribe to `nav_msgs::OccupancyGrid` from `week5` node
2. In the callback, use a `tf::TransformListener` to find the current location
3. Create for loop from _i_=-4 to 4.
    - Create another for loop inside that iterates form t=0 to some `num_steps` variable that you create
        - For each t, calculate where `oswin` would be if it moved at that angular velocity and a linear velocity of 1.
        - Calculate the cost, add that to the total sum for that _i_
4. Find the i that with the lowest cost
5. Take that angular velocity, put it in a `geometry_msgs::Twist` message, and then publish it on the `oswin/velocity`
topic

----

- That's it! Hopefully you'll see `oswin` plan a nice path
- Be sure to make use of `rviz`
    - Display `nav_msgs::Path` to "see" what the robot is thinking
- Can you make it plan better?

---

## Improving the simple planner

The markdown version contains an improvement that you can make to the simple planner

---

## Summary

----

- Path Planning
    + Planning a path form start to goal while avoiding obstacles
    + Three main paradigms: Grid-based, sampling-based, optimization-based

---

## Next Week
Next week, we'll look at doing computer vision with **OpenCV** to detect different things from **camera data**.

---

See you next week!
