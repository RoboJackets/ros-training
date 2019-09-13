# Week 6
Welcome to Week 6 of ROS training exercises! We'll be building off of last week's **mapping** and
learn about simple **path planning**.

## Overview of Path Planning
Path planning, as the name suggests, is the act of planning a path. In the context of robotics, most of the time
there is a **start** from which you are planning from (the robot's current location), and a **goal** that you are
trying to plan towards.

There are three main paradigms in which path planning can be performed:

#### **Grid-based search**
Traditional grid-based algorithms pretend that the map is a grid, and then it simply searches through this grid for
a path from the start node to the goal node.

Examples of this paradigm include [A*](https://en.wikipedia.org/wiki/A*_search_algorithm)
and [D*](https://en.wikipedia.org/wiki/D*).

#### **Sampling based**
Sampling based path planning algorithms work by randomly sampling points in the **configuration space** and connecting
previous sampled points to new sampled points, until the goal is found.

The most famous example of this paradigm would be [RRT](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree).

#### **Optimization based**
The last paradigm of path planning is optimization based algorithms. These algorithms interpret path planning as an
optimization / controls problem, where you have a set of states **x**, a set of control actions **u** that act on these
states, and some cost function **f(x, u)** which assigns states and control actions some cost.

A path is then formed by finding a set of **x** and **u** that minimize the cost function. 

Don't worry too much about how this optimization is actually done - this is a pretty complex topic. Check out
[this book](https://web.stanford.edu/~boyd/cvxbook/bv_cvxbook.pdf) if you're interested in learning more about
optimization.

An example of this would be [TEB Local Planner](http://wiki.ros.org/teb_local_planner), a popular trajectory planner
package in ROS.

## Implementing a (simplified) optimization based path planner
Let's implement a very simplified optimization based path planner in
[src/week6/main.cpp](../igvc_training_exercises/src/week6/main.cpp), where the objective is to have the `oswin` turtle
navigate to where the `kyle` turtle is, without hitting any of the obstacles in between.

To do path planning though, we need a **map**. But wait, you guys already wrote
a node that creates a map [last week](week5.md). We'll be making use of that.

The path planner will work by looking at 9 paths that have different **angular** velocity from -4 to 4. For each
different angular velocity, calculate where the resulting pose after having driven using that angular velocity for a
certain **Δt** (a parameter) seconds, and then calculate the cost of each resulting pose. Pick the angular velocity
that resulted in the lowest cost, and publish the `geometry_msgs::Twist` message to `oswin/velocity`.

For now, the cost function will be the current distance of `oswin` to `kyle`. However, if the resulting pose is on top
of an obstacle in the map, then the cost will be `std::numeric_limits<double>::infinity()`, as we don't want the turtle
to hit any obstacles.

Here's the plan:
1. In the week6 `main.cpp`, subscribe to the `nav_msgs::OccupancyGrid` that the `week5` node is publishing
2. In the callback for the `week6` node, use a `tf::TransformListener` to find out the current location of `oswin` and
`kyle`.
3. Create a for loop that iterates from i=-4 to 4.
    - Create another for loop inside that iterates form t=0 to some `num_steps` variable that you create
        - For each t, calculate where `oswin` would be if it moved at that angular velocity and a linear velocity of 1.
            Calculate the cost for each resulting pose using the cost function, and add that cost to a sum
            for that i
4. Calculate the i that results in the lowest cost
5. Take that angular velocity, put it in a `geometry_msgs::Twist` message, and then publish it on the `oswin/velocity`
topic

Hopefully you will be able to see `oswin` plan a nice path and be able to reach `kyle` without hitting any obstacles.

And that's it! You've implemented a very simple optimization based path planner.

### Adding more stuff to the path planner
Now, there are plenty of things that can be added to this path planner.

The first (and most important imo) thing that needs to be added is **visualization**. Right now, you have no idea what
the path you planned looks like. An easy way of doing that is to publish `nav_msgs::Path` messages, which rviz is able
to show. To do this, you can take the best angular velocity but apply it in much smaller time increments to generate a
bunch of `geometry_msgs::PoseStamped`, and then insert that into the `nav_msgs::Path` and publish it.

It should look something like this:
![](training_nav_rviz.png)

We then have a few other things that can be added to this simple path planner:

<details>
  <summary>1. Executing two controls</summary>
  
  So far, our optimization based path planner assumes that we're executing one command
  (ie. one linear and angular velocity) for the entire path. However, this isn't a very good assumption. For example,
  
  ![](example_one_control.png)
  
  It's very clear that the current "optimal" path isn't optimal at all, as intuitively you'd want the path to turn
  right first to avoid the barrel, then turn left in order to head towards the goal.
  
  What we can do is **optimize over multiple controls** now instead of just one. For now, we'll increase the number of
  controls we're executing, so that we optimize one control for half the time, and then another one for the rest of the
  time.
  
  One way of doing this is to define a `std::vector<double>` for a "pair of controls", and then generate all the
  possible controls in a `std::vector<std::vector<double>>`, and then loop over those controls to find the one that
  minimizes the cost. 
  
  This should look something like this (see how nice visualization is):
  ![](two_controls.png)
</details>

<details>
  <summary>2. Executing more than two controls</summary>
  
  Two controls does a better job than one control, but its still pretty bad. What we really want is to be able to
  execute a series of controls, one for every timestep.
  
  However, there's a problem if we repeated what we did for two controls but for say 10: The **search space** increases
  dramatically.
  
  What does **search space** refer to?
  
  With one control, we're searching for **one** number. In reality, the number we're looking for lies on the real number
  line, and so there are an _infinite_ number of controls we're searching through, but because we **discretize** our
  search space to 9 different possibilities, we're only looking for the best one out of these 9.
  
  With two controls, there's 9 different possibilities for the first control, and 9 different possibilities for the
  second. Using basic combinatorics, that's a total of 9^2 = 81 different options that we need to search through.
  
  If we've got 10 different controls, then that's a total of 9^10, or around 3 billion different controls that we need
  to search through. That's way too many to do in a reasonable time.
  
  This is where optimization algorithms come in.
</details>

## Summary
That's it for this week!

We learnt about:
- [An overview of path planning](#overview-of-path-planning)
    - **Grid-based search**: Search through the grid for a path
    - **Sampling based**: Randomly sample poses and connect them to find a path
    - **Optimization based**: Treat the problem as an optimization problem with constraints
- [Implemented a simple optimization based](#implementing-a-simplified-optimization-based-path-planner)
    - Learning the kinematics for a differential drive robot
    - Hopefully it's able to plan around the obstacles
    - Publishing a debug path to help with visualizing the algorithm

Next week, we'll be learning about **images manipulation** with **OpenCV** and how to use that
to **identify objects** in our images.
