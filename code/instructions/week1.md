# Week 1
Welcome to Week 1 of ROS training exercises! I will first give a brief overview of what ROS is,
then we will learn the basics of ROS nodes and learn how to launch nodes.

## What is ROS?
ROS stands for Robotic Operating System. It's a very convenient framework for programming robots.

That was a short and probably not very useful explanation of what ROS is, but throughout
the coming exercises we'll learn more about what ROS is by using its core features, and
hopefully you'll gain a better understanding of what it is, but for now:

## How does ROS work?
The core of ROS deals with multiple small programs, called __nodes__, which communicate
with each other. The important thing is that each node is only responsible for one aspect
of the robot, which helps make the entire stack much cleaner.

For example, we might have a
nodes for identifying barrels, mapping, localization (finding out where the robot is) and
path planning.

## ROS Nodes and Launching Buzzsim
Before we write our own node, let's first launch an existing node. Because the ROS ecosystem
is quite large, we can easily use ROS nodes which other people have written for our own projects.

The first thing we need to do is run `roscore`:
```bash
roscore
```

Don't worry about what `roscore` is. You can think of it as something that handles all the ros functionality behind the
scenes. Without launching `roscore` first, you'll see an error along the lines of:
```
[ERROR] [1567021515.828919061]: [registerPublisher] Failed to contact master at [localhost:11311].  Retrying...
```

Let's start by launching the `buzzsim` simulator that we will make use of the coming few weeks. Run this in another
window:
```bash
rosrun igvc_buzzsim buzzsim
```
Here, `buzzsim` is the name of the **executable**, while `igvc_buzzsim` is the name of the
**ROS package** which the `buzzsim` executable is from.

You can think of a **ROS package** as an organizational unit under which we can put ROS nodes,
libraries, scripts and more.

If you get the error:
```
[rospack] Error: package 'igvc_buzzsim' not found
```

then this means that you have not sourced the file at `catkin_ws/devel/setup.bash` (or `catkin_ws/devel/setup.zsh` if
you're using zsh):
```bash
cd catkin_ws # Or wherever your catkin_ws is located
source devel/setup.bash
```

You can do the following to have the command run for you automatically, replacing `catkin_ws` with the path to where
your `catkin_ws` is located.
```bash
echo "source catkin_ws/devel/setup.bash" >> ~/.bashrc
```

After launching the node, you should see the simulator window open up:
![buzzsim](buzzsim.png)

## ROS Topics and Keyboard Teleop
Now that we've got the simulator up and running, let's try moving the turtle around. To do that, we
need to communicate to the simulator node. ROS does this through the use of **topics** and **messages**.

You can think of a ROS topic as an address that messages are sent to, and ROS messages as the actual messages
that are sent to those topics.

For example, the simulator node listens to messages on the `/oswin/velocity` topic, makes the turtle move
depending on what message was sent. This means that we can control the turtle by sending a message on the
`/oswin/velocity` topic.

<img src="https://drive.google.com/uc?export=download&id=1eMJvdkT4IcLWoMIsSyF7P_bzkNNJW8yU">

Let's try this. Open up another terminal window, and launch the `teleop_twist_keyboard` node like so:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity
```

Ignore the details of the syntax for now, but this launches the `teleop_twist_keyboard` node from the
`teleop_twist_keyboard` package. You should see something like this in the terminal window:
![keyboard_teleop](keyboard_teleop.png)

Try pressing the keys in the image. You should see the turtle in the simulator start to move around.

What's happening is that the `teleop_twist_keyboard` node **publishes** the velocity commands to the
`/oswin/velocity` topic and because `buzzsim` node **subscribes** to the same topic, it is able to
receive those messages and move the turtle. We call the `teleop_twist_keyboard` a **ROS Publisher**
and the `buzzsim` node a **ROS Subscriber**.

<img src="https://drive.google.com/uc?export=download&id=1IMTA2zjoZJ-V2lv87u_clImAZ7xkhvEK">

## The `rostopic` tool
To look more into this, we can use the `rostopic` tool. Close the `teleop_twist_keyboard` node with
Ctrl-C, and type the following:
```bash
rostopic --help
```

The help screen of `rostopic` should show, listing out the different commands that are possible with
`rostopic`. Let's try out the `list` options:
```bash
rostopic list
```

You should see a few topics:
```
/oswin/velocity
/rosout
/rosout_agg
/tf
```

Notice that the `/oswin/velocity` topic is shown here. We can get more information about a ROS topic
with the `info` command:
```
Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /buzzsimsim (http://{YOUR USER NAME HERE}:36899/)
```

Here, we can see that this topic is of type `geometry_msgs/Twist`, has no `Publishers`, and has one `Subscriber`,
which is the `buzzsim` node. The type of a topic is the type of message of that topic. In this case, it is
`geometry_msgs/Twist`, which contains information about the velocity of the robot.

Since `geometry_msgs/Twist`, which comes from the package `geometry_msgs` package from ROS, we can
lookup the definition in the [ROS docs](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html)
(Google "geometry_msgs/Twist" and it should show up)

<img src="https://i.imgur.com/uovZmPe.png">

### `rostopic echo`
We can see exactly what is being sent on a topic with the `echo` command:
```bash
rostopic echo /oswin/velocity
```

You should see nothing being printed out: That's because nothing is being published to the this topic right now.
Let's publish something to this topic. Open up another terminal window, and open up the `teleop_twist_keyboard` node
again: 
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/oswin/velocity
```

This time when you move the turtle around using the keyboard teleop, you should see the exact messages being published
in the terminal window with `rostopic echo`, something like:
```
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

### `rostopic pub`
We can also publish the messages directly from the command line also, using the `pub` command. Close the `teleop_twist_keyboard`
node again, and then run the following:
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

Note that tab completion is really useful here. Type `rostopic pub /oswin/velocity ` and then keep tabbing. Also,
the identation matters (for those of you who know YAML, this is YAML).

The `rostopic pub` command will publish a message of type `geometry_msgs/Twist` to the `/oswin/velocity` topic,
and you should now see the turtle begin to move forward. To stop it, run the same command, but with a `0` for the
linear `x` velocity instead:
```bash
rostopic pub /oswin/velocity geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

Try playing around with the the `x` part of `linear` and the `z` part of `angular`, and seeing how the turtle
moves.

## ROS messages
Now that we've seen how to use the `rostopic` tool, let's look more at the ROS **messages** themselves. So far, we've
seen the `geometry_msgs/Twist` message, but how do they work?

### `.msg` files
ROS Messages are defined using `.msg` files. For example, the
[`twist.msg`](https://github.com/ros/common_msgs/blob/jade-devel/geometry_msgs/msg/Twist.msg) looks like:
```.msg
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular
```

Each line represents one field in the message, with the type on the left and the
name on the right. In this case, the `twist.msg` message has two fields:
"linear" of type `Vector3`, and "angular" of type `Vector3`. The `Vector3` type that
this message refers to is another type that's defined
[somewhere else](https://github.com/ros/common_msgs/blob/jade-devel/geometry_msgs/msg/Vector3.msg):
```.msg
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
```

In general, messages are described by other messages within them.

### Exercise: Writing our own ROS message
Let's try writing our own ROS message now. Imagine that want to write a path planner node that
publishes motor commands, and a motor controller node that subscribes to the motor commands.

We don't have a ROS message type for motor commands yet though, and so we need to make our own
custom ROS message.

Some requirements for our message:
1. Our robot has two motors, one for each wheel, so we need to be able to send each motor a separate command
2. We want to know **when** the message was sent, so we can tell if the message is old or not.
3. The command sent to the motor should be a float

Write your ROS message in the file
[`igvc_training_msgs/msg/motor_command.msg`](../igvc_training_msgs/msg/motor_command.msg).

Tips:
- Check out the [std_msgs](http://wiki.ros.org/std_msgs). It should contain everything you need

Remember to `catkin_make` to compile the message!

Try doing `rostopic pub` and `rostopic echo` to verify that your message works.

## Summary
And that is it for week 1 of the exercises.

This week, we learnt about:
- [Launching ROS Nodes](#ros-nodes-and-launching-buzzsim)
    + `rosrun <package-name> <node-name>`, ie. `rosrun igvc_buzzsim buzzsim`
- [ROS Topics and `teleop_twist_keyboard`](#ros-topics-and-keyboard-teleop)
    + A ROS topic is like an address that messages are sent to
    + A ROS node can **subscribe** and **publish** messages to any topic
    + Each topic has a message type.
- [The `rostopic` tool](#the-rostopic-tool)
    + `rostopic list` to list available topics
    + `rostopic echo` to listen to topics
    + `rostopic pub` to publish to topics
- [ROS messages](#ros-messages)
    + Describing a ROS message with a `.msg` file
    + Writing our own ROS message by using messages from `std_msgs`

In [week2](week2.md), we'll be looking at how to write **ROS Publishers**
and **ROS Subscribers** in C++.
