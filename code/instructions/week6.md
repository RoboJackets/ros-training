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
3. Create a for loop that iterates from i=-4 to 4. For each i, calculate where `oswin` would be if it moved at that
angular velocity and a linear velocity of 1. Calculate the cost function for each resulting pose
4. Calculate the angular velocity that results in the lowest cost
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
