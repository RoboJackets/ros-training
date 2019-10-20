---
title: Week 4
revealOptions:
    width: 1600
    height: 900 
---
# Week 4

---
## Recap of last week

----

#### Control Theory

control a system to do what we want <!-- .element: class="fragment" data-fragment-index="1" --> 

----

What's one example of a controller that we learned last week?

PID <!-- .element: class="fragment" data-fragment-index="1" --> 

----

What does PID stand for?

Proportional Integral Derivative <!-- .element: class="fragment" data-fragment-index="1" --> 

----

What is a proportional controller?

A controller where the control effort is proportional to the error <!-- .element: class="fragment" data-fragment-index="1" --> 

----

What is a integral controller?

A controller where the control effort is proportional to the integral of the error <!-- .element: class="fragment" data-fragment-index="1" --> 
 
----

Why do we need an integral controller?

A proportional controller cannot deal with steady state error. <!-- .element: class="fragment" data-fragment-index="1" --> 

----

What is a derivative controller?

A controller where the control effort is proportional to the derivative of the error <!-- .element: class="fragment" data-fragment-index="1" --> 

----

Why do we need a derivative controller?

To reduce overshoot from the controller <!-- .element: class="fragment" data-fragment-index="1" --> 

----

How is `roslaunch` different from `rosrun`?

<div><pre class="inline">roslaunch</pre> allows you to launch multiple nodes at once</div> <!-- .element: class="fragment" data-fragment-index="1" --> 

<div><pre class="inline">roslaunch</pre> allows you to define parameters for nodes</div> <!-- .element: class="fragment" data-fragment-index="2" --> 

----

How do you define a parameter?

`nh.getParam(param_name, variable)` <!-- .element: class="fragment" data-fragment-index="1" --> 

`<param name="param_name" value="value" />` <!-- .element: class="fragment" data-fragment-index="2" --> 

----

How would you visualize messages on a graph?

`rqt_plot` <!-- .element: class="fragment" data-fragment-index="1" --> 

---

Welcome to Week 4 of ROS training exercises!

- IMU
- Coordinate Frames
- Visualization with `rviz`
- `tf`

---

## IMU

Inertial Measurement Unit

----

- Sensor used for finding orientation
- A 9 DOF (degree of freedom) IMU measures 9 things
    - **Accelerations** (x, y, z) with **acceleromteter**
    - **Angular velocity** (x, y, z) with **gyroscope**
    - **Magnetic Field** (x, y, z) with **magnetometer**
    
----

Find **acceleration**, **angular velocity** and **heading**

---

## Coordinate frames and the IMU

**Coordinate frame** are important when working with sensors

----

A **coordinate frame** refers to the coordinate system that is used

----

- For example, the IMU can measure accelerations in the x, y and z axes in the **robot**'s coordinate frame
- This is different from the **world**'s coordinate frame.

----

An example:
- IMU is mounted in the same direction as my robot
- When the IMU records an acceleration in the X direction, it means that the robot has accelerated forwards
in the +X direction (cyan arrow).

<p align="center">
    <img src="https://i.imgur.com/MiS9UAU.png" />
</p>

----

- Robot is located at (2,1), facing north
- **Robot**'s coordinate frame
    - +X acceleration
    - (1, 0, 0)
- **World**'s coordinate frame
    - +Y acceleration
    - (0, 1, 0)

<p align="center">
    <img src="https://i.imgur.com/MiS9UAU.png" />
</p>

----

#### Note
ROS has a [REP 103](https://www.ros.org/reps/rep-0103.html) standard
- Defines conventions for coordinate frames
- Right hand coordinate frame
- For bodies (robobts)
    - X points forward, Y points left, and Z is pointing up
- For geographic locations
    - X points east, Y points north, and Z points up.

----

## Localization with dead reckoning

- In robotics, we need to figure out **where** we are
- Called **localization**

----

- Can perform localization with an IMU
- Simplest form of localization is called **dead reckoning**

----

### Dead Reckoning

- Determine position by taking your previous position and integrating your velocity over the
elapsed time
- For IMU, **linear accelerations** instead of **linear velocities**
    - Need to integrate **twice**
    - Keep track of our velocities.
    
----

This means that our **state**, the things we need to keep track of, consists 4 things
- x
- y
- heading
- x velocity.

----

Why only x velocity? What about y velocity?

Because our robot is a differential drive, we can't move sideways, so y velocity is zero <!-- .element: class="fragment" data-fragment-index="1" --> 

<p><b>Which coordinate frame</b> is the x velocity in?</p> <!-- .element: class="fragment" data-fragment-index="2" --> 

The robot's frame <!-- .element: class="fragment" data-fragment-index="3" --> 

----

How do we use the IMU measurements to update our **state**?

Reminder: IMU gives us x acceleration, angular velocity, heading

Hint: Physics <!-- .element: class="fragment" data-fragment-index="3" --> 

----

Use kinematics equations:

<img class="eqn-big" src="https://latex.codecogs.com/svg.latex?\begin{align*}&space;v_{t&plus;1}&space;=&&space;v_{t}&space;&plus;&space;a_{t}&space;\,&space;dt&space;\\&space;\theta_{t&plus;1}&space;=&&space;\theta_{t}&space;&plus;&space;\omega_{t}&space;\,&space;dt&space;\\&space;v_{average}&space;=&&space;\frac{v_{t&plus;1}&space;&plus;&space;v_{t}}{2}\\&space;x_{t&plus;1}&space;=&&space;x_{t}&space;&plus;&space;cos(\theta_{t&plus;1})&space;\,&space;v_{average}&space;\,&space;dt&space;\\&space;y_{t&plus;1}&space;=&&space;y_{t}&space;&plus;&space;sin(\theta_{t&plus;1})&space;\,&space;v_{average}&space;\,&space;dt&space;\\&space;\end{align*}" title="\begin{align*} v_{t+1} =& v_{t} + a_{t} \, dt \\ \theta_{t+1} =& \theta_{t} + \omega_{t} \, dt \\ v_{average} =& \frac{v_{t+1} + v+{t}}{2}\\ x_{t+1} =& x_{t} + cos(\theta_{t+1}) \, v_{average} \, dt \\ y_{t+1} =& y_{t} + sin(\theta_{t+1}) \, v_{average} \, dt \\ \end{align*}" />

----

- Many ways of numerically integrating the data that yield more accurate results
- This is just one simple way.
- By doing this, we should be able to keep track of our position.

---

### Exercise: Implementing dead reckoning using an IMU in ROS
Launch the environment with `roslaunch`:
```bash
roslaunch igvc_training_exercises week4.launch
```

You should see just a single turtle in the middle:

----

Let's look at what topics are published. How do I find that out?

`rostopic list` <!-- .element: class="fragment" data-fragment-index="3" --> 

----

- There's now a topic `/oswin/imu` of type `sensor_msgs/Imu`.
- Also, you can see that `/oswin/ground_truth` isn't there
- We'll need to use dead reckoning to compute an estimate of our pose.

----

Let's graph `/oswin/imu` and see what we get

- Open up `rqt_plot` and graph the accelerations in the x axis
- Open up teleop_twist_keyboard as before:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity
```

Move around a bit and get a feel for what the accelerations from the IMU look like as you move around.

----

Its time to start writing some code.
We'll be implementing this node in [week4/main.cpp](../igvc_training_exercises/src/week4/main.cpp).

----

What's our plan for writing this node?

<ol>
<li>Create a subscriber to <code class="inline">/oswin/imu</code></li> <!-- .element: class="fragment" data-fragment-index="1" --> 
<li>Create a publisher to <code class="inline">/oswin/odometry</code></li> <!-- .element: class="fragment" data-fragment-index="2" --> 
<li>Implement dead reckoning in the callback</li> <!-- .element: class="fragment" data-fragment-index="3" --> 
<li>Publish the estimated position</li> <!-- .element: class="fragment" data-fragment-index="4" --> 
</ol>

----

#### 1 & 2: Create the publisher and subscriber

`ROS_INFO_STREAM` the x acceleration to make sure everything is working.

----

#### 3. Implement Dead Reckoning

- Before we start, we need to setup the global variables representing our **robot state**
    - x, y, x velocity, heading
- Also need to keep track of the **time** which we received the last message
    - To calculate the delta t for the kinematics
 
----

Kinematics

<img class="eqn-huge" src="https://latex.codecogs.com/svg.latex?\begin{align*}&space;v_{t&plus;1}&space;=&&space;v_{t}&space;&plus;&space;a_{t}&space;\,&space;dt&space;\\&space;\theta_{t&plus;1}&space;=&&space;\theta_{t}&space;&plus;&space;\omega_{t}&space;\,&space;dt&space;\\&space;v_{average}&space;=&&space;\frac{v_{t&plus;1}&space;&plus;&space;v_{t}}{2}\\&space;x_{t&plus;1}&space;=&&space;x_{t}&space;&plus;&space;cos(\theta_{t&plus;1})&space;\,&space;v_{average}&space;\,&space;dt&space;\\&space;y_{t&plus;1}&space;=&&space;y_{t}&space;&plus;&space;sin(\theta_{t&plus;1})&space;\,&space;v_{average}&space;\,&space;dt&space;\\&space;\end{align*}" title="\begin{align*} v_{t+1} =& v_{t} + a_{t} \, dt \\ \theta_{t+1} =& \theta_{t} + \omega_{t} \, dt \\ v_{average} =& \frac{v_{t+1} + v+{t}}{2}\\ x_{t+1} =& x_{t} + cos(\theta_{t+1}) \, v_{average} \, dt \\ y_{t+1} =& y_{t} + sin(\theta_{t+1}) \, v_{average} \, dt \\ \end{align*}" />

----

- Publish this information so that we can visualize it, and so that other nodes are able to use this information
- ROS provides the `nav_msgs::Odometry` message type that describes the robot's current location.

----

What is a Quaternion?

<img src="https://i.imgur.com/wNsz2wL.png" />

----

#### Representing orientations: euler angles and quaternions

- The `orientation` field uses something called **quaternions**
- Use 4D **unit vector** to represent orientations in 3D space
- If you're interested in the math of how quaternions work, feel free to
google
    - There's a neat [video](https://www.youtube.com/watch?v=zjMuIxRvygQ) by 3Blue1Brown that explains it.

----

- Don't worry if you don't understand how they work though
- We can work with another representation of orientations called **euler angles** which are much
simpler to understand

----

**Euler Angles** uses three numbers: **roll**, **pitch** and **yaw**

If you're familiar with airplanes then this should be familiar to you

----

- **roll** represents rotation around the **x** axis
- **pitch** represents rotation around the **y** axis
- **yaw** represents rotation around the **z** axis.
- Right hand rule


<p align="center">
    <img src="https://www.touringmachine.com/images/PitchRollYaw.png">
</p>

----

- Because we're working in 2D, **roll** and **pitch** are 0
- Only need to worry about is **yaw**.

----

#### Converting between euler angles and quaternions

In order to convert between euler angles and quaternions, the ROS
built-in `tf` package provides a few useful methods that we can use.

----

Add the following `#include`:

```c++
#include <tf/transform_datatypes.h>
```

We can then use the `tf::createQuaternionMsgFromYaw` message to convert from yaw to a quaternion. This might look
something like:
```c++
odometry_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(g_heading);
```

----

#### The `Header` message type
- One important field that we haven't gone over yet is the `Header` field on `nav_msgs::Odometry`
- If you've finished the Integral and Derivative controller from last week, then you probably have seen this before

----

Let's look at the definition of the message from the
[ros documentation](http://docs.ros.org/melodic/api/std_msgs/html/msg/Header.html)

<p align="center">
    <img src="https://i.imgur.com/ju4N22Y.png" />
</p>

- `stamp` refers to the timestamp of the message.
- `frame_id` on the other hand refers to the name of the **coordinate frame** of the message

----

- x acceleration is different depending on which coordinate frame you're talking about
- Use `frame_id` to be clear about which **coordinate frame** we are talking about

----

If our world frame was called `yeet`, and we wanted to say that the position of our robot was in relation
to the `yeet` frame
```c++
nav_msgs::Odometry msg;
msg.header.frame_id = "yeet";
```

----

Of course, don't actually call your coordinate frame `yeet`
- ROS defines a few standard coordinate frame names in 
[REP 105](https://www.ros.org/reps/rep-0105.html)
- We will be using the `odom` frame, which is a world-fixed frame:

```c++
nav_msgs::Odometry msg;
msg.header.frame_id = "odom";
```

---

#### Visualizing our localization algorithm with `rviz`
- We're finished with the algorithm, but we have no idea how well it works
- Visualization is the most important thing when debugging
- To visualize the `nav_msgs::Odometry` message, we can use the `rviz`
    - Tool for visualizing the robot and other informations.

----

Open up rviz by running the `rviz` command:
```bash
rviz
```

----

You should see something similar to the screen below:

<img src="https://i.imgur.com/dzpbhr3.png" />

----

In order to visualize the `nav_msgs::Odometry` message that we're publishing:
1. On the left in the `Global Options` section change the `Fixed Frame` to "odom", the value of the `frame_id` we set
earlier.
2. On the right, in the `Views` section, change the `Type` to `TopDownOrtho` to change the view from 3D to top down 2D.
3. Click the `Add` button on the bottom left, select the `By topic` tab, and click `Odometry` under `/oswin/odometry`.

----

You should now see a red arrow in the middle of your screen visualizing the `nav_msgs::Odometry` message that you're
publishing.

<img src="https://i.imgur.com/gndFkyd.png" />

----

Try moving the robot again using `teleop_twist_keyboard`
- Both the turtle in the simulator and the red arrow move
- Hopefully the red arrows will be in the same location as the turtle in the simulator

----

#### `tf` and tf::TransformBroadcaster
We're almost done

The last thing we need to do is to "broadcast" our transform (the position of our robot) to `tf`

---

What is `tf`? We `#include`'ed a header file from `tf` earlier.

> tf is a package that lets the user **keep track of multiple coordinate frames over time**. tf maintains the
> relationship between coordinate frames in a tree structure buffered in time, and lets the user **transform points**,
> vectors, etc **between any two coordinate frames** at any desired point in time.  

---

- Nifty library for managing transforms
- We've seen at the beginning how annoying coordinate frames are
- `tf` will allow us to easily express data from _one coordinate frame_ in _another coordinate frame_

----

- `tf` needs to know where these coordinate frames are in
- We can do that by **broadcasting** the transform
- To do that, we need to do three things:

----

##### 1. Create a `tf::TransformBroadcaster`
Create a `tf::TransformBroadcaster`:
```c++
tf::TransformBroadcaster transform_broadcaster;
```

----

#### 2. Create a `tf::TransformStamped` to hold the transform
- We need to create a `tf::TransformStamped` variable in order to pass it to the `tf::TransformBroadcaster` that we just
made
- Unfortunately, since this isn't a ROS message, this can't be looked up as easily as other messages

Thankfully, Clion is here to save the day

----

- Type in `tf::TransformStamped`, and then move your cursor over it
- Hit Ctrl-Alt-B (Or Ctrl-Shift-A and search "go to defintion") to go to the definition of `tf::TransformStamped`
- Find the **constructor** (to see what fields we can pass it):

```c++
StampedTransform(const tf::Transform& input, const ros::Time& timestamp, const std::string & frame_id,
                 const std::string & child_frame_id):
    tf::Transform (input), stamp_ ( timestamp ), frame_id_ (frame_id),
    child_frame_id_(child_frame_id)
{ };
```

----

We need to pass **4** things:
1. `tf::Transform` input
2. `ros::Time` timestamp
3. `std::string` frame_id
4. `std::string` child_frame

----

`tf::Transform` represented the actual **transform** between the two coordinate frames

Two useful methods that we need:

----

##### `.setOrigin(tf::Vector3 origin)`

- takes in an argument of type `tf::Vector3`
- Sets the **origin**, or **position** of the transform
- Don't worry, `tf::Vector3` has a sane constructor - you pass in x, y and z coordinates
in the constructor:

```c++
tf::Transform transform;
tf::Vector3 origin(1.0, 2.0, 3.0);
transform.setOrigin(origin);
```

----

##### `.setRotation(tf::Quaternion quaternion)`
- Takes in an argument of type `tf::Quaternion`
- Sets the **rotation** of the transform

----

But theres one problem:
- Oh noes, its in quaternions and not yaw!
- We can just do the same thing as we did earlier
    - `tf::createQuaternionFromYaw` instead of `tf::createrQuaternionMsgFromYaw`
    
```c++
tf::Transform transform;
transform.setRotation(tf::createQuaternionFromYaw(1.57));
```

----

Next, `ros::Time` timestamp

- Time at which this transform was recorded

Why does there need to be a timestamp?

----

- For **static transforms**, ex. transform from the imu to your robot
    - the timestamp doesn't matter
- For other transforms, ex. transform from the world frame to your robot AKA your robot's position
    - transform is different at different moments in time
- We'll make use of this next week.
- We can just use the timestamp of the IMU message that we received.

----

Finally, the `frame_id` and `child_frame` arguments of type `std::string`
- Names of the frame and child frame for the transform you're trying to publish

----

For example, trying to publish the transform of **the robot frame** in the **world frame**

- `frame_id` (the reference frame we're currently in) is `odom`
- `child_frame` (the reference frame we're trying to describe) is `oswin`.

----

#### 3. Call the `.sendTransform` function on the `tf::TransformBroadcaster`
Finally, we call the `.sendTransform` function on the `tf::TransformBroadcaster` we created earlier.

----

That's it....... except not really.
Most likely you've created `tf::TransformBroadcaster` in one of two ways:
1. Created it as a global variable
2. Created it in the callback

Both of these ways don't work

----

We can verify this by running `rosrun tf tf_monitor`, a tool which prints out all the
coordinate frame transforms that `tf` receives

If it were working, you would see something like:
```text
Frame: oswin published by unknown_publisher Average Delay: 0.000244823 Max Delay: 0.000300968
```

But it's empty. What's the problem?

----

If you created it as a global variable
- `You must call ros::init() before creating the first NodeHandle`
    - The constructor of `tf::TransformBroadcaster` creates its own `ros::NodeHandle`
    - This has to happen after `ros::init()`

----

If you created it in the callback, no error shows, but it still doesn't work.

- Problem: `tf::TransformBroadcaster` needs some time to send the transform
    - Similar to `ros::Publisher` during the first week
    - We can't wait here though, since we need the transforms as soon as possible
    - We want to know where we are **right now**, not like 1 second ago

----

Want to be able to create the `tf::TransformBroadcaster` **after** we do `ros::init`

Only create it **once** and not everytime

----

How can we fix this problem?

----

We can do this with **static local variables**

---

Adding the **static** keyword to a **local variable**:
- It will only be initialized **once** the **first time** you come acros the statement
- Won't be destroyed when the scope ends
- When you come across the statement again, the variable is already initialized again and doesn't
get initalized again
- Have access to the **same** instance of the variable.

----

So, if we created `tf::TransformBroadcaster` as a **static** variable inside the callback like so:
```c++
void imuCallback(sensor_msgs::Imu msg)
{
  ...
  static tf::TransformBroadcaster transform_broadcaster;
}
```

- **First** time the callback is called, which is **after** we do `ros::init`, the `tf::TransformBroadcaster`
variable will be initialized
- **Next** time the callback is called, the **same** instance of the variable will still be there
    - Enough time to send the transform
    
----
    
You should see that `rosrun tf tf_monitor` is able to show that the `oswin` frame is getting published to by your node

----

#### Visualizing `tf` frames in rviz
- We can visualize the `tf` frame we published to in `rviz`
- Open up `rviz`, and then add a display of type `tf`.
- You _should_ be able to see two frames show up on the screen: "odom" and "oswin"

----

- Verify that `tf::TransformBroadcaster` is working properly by moving the turtle around
- The "oswin" frame should also be moving.

----

## Summary
And that's it for this week!

----
- [The IMU](#imu)
    + Measures **acceleration**, **angular velocity**, and **orientation**
    + Has an **accelerometer**, **gyroscope** and **magnetomer**
----
- [Coordinate Frames](#coordinate-frames-and-the-imu)
    + Coordinate Frames defines a set of direction relative to some object
    + Understanding the basics of the REP 103 standards for Coordinate Frames
----
- [Localization with Dead Reckoning](#localization-with-dead-reckoning)
    + Using kinematics equations to locate ourselves
----
- [Implemented dead reckoning in ROS](#exercise-implementing-dead-reckoning-using-an-imu-in-ros)
    + `nav_msgs::Odometry` message type for odometry information
    + Turned something theoretical to something that was working again!
----
- [rviz](#visualizing-our-localization-algorithm-with-rviz)
    + Use `rviz` to visualize what's happening
----
