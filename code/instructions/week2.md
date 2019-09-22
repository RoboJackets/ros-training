# Week 2
Welcome to Week 2 of ROS training exercises! We'll be learning how to write **ROS Publishers** and **Subscribers**
with C++.

## Writing a ROS Subscriber and Publisher in C++
We've looked at how we can publish messages, now it's time to do it in C++. Before we do that though, let's
figure out how we can write a node in C++ first.

### Hello World with ROS and C++
Let's start by writing Hello World with ROS and C++. There is standard a standard Hello World program in
[code/igvc_training_exercises/src/week2/publisher.cpp](../igvc_training_exercises/src/week2/publisher.cpp):
```C++
#include <iostream>

int main(int argc, char** argv)
{
  std::cout << "Hello World!" << std::endl;
}
```

We've already compiled this when compiling last week, but just as a recap, we can compile everything in the
catkin workspace by running
```bash
cd catkin_ws # cd to where your catkin workspace is located
catkin_make
```

Now, try running the executable with `rosrun`. The ROS package is called `igvc_training_exercises`, and the node is
called `week2_publisher`. You can refer back to [week 1](week1.md) on the details of the command if you forgot. Otherwise,
here's the [answer (Hover over me)](#spoiler "rosrun  igvc_training_exercises  week2_publisher").

Verify that Hello World correctly prints out:
```
Hello World
```

Although we have a Hello World executable, this isn't a node yet. To make this a proper ROS node, we need to initialize
it.

Start by adding an *include* for the ROS headers at the top of the file. Don't worry about exactly what this does right now, as we'll be covering
this in general software training later.

```c++
#include <ros/ros.h> // <-- Add this line

#include <iostream>
...
```

Next, add the following line above the `std::cout`:
```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2"); // <-- Add this line

  std::cout << "Hello World!" << std::endl;
}
```

`ros::init` initializes a ROS node, and the last argument in the function is the name of the node. To verify that it
works, save the file and recompile with `catkin_make`. You should still see `Hello World` being printed out. However, we
can't actually tell that it's a node because the program is exiting right after it prints `Hello World`. To make the
program wait, add this line after `std::cout`:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");

  std::cout << "Hello World!" << std::endl;
  ros::spin(); // <-- Add this line
}
```

For now, don't worry about what `ros::spin()` does. All you need to know is that it stops the program from exiting until
you do Ctrl-C to stop it.

Compile the program again, and run it. Now, you should see that the program doesn't exit. Open up a new terminal window,
and type in `rosnode list`. You should see the `week2` node show up:
```
/rosout
/week2
```

### Writing a simple publisher
Now, let's write a simple publisher that publishes a number. To do that, we need to do four things:

#### 1. Create a `ros::NodeHandle`

To create a Publisher, we first need to create a `ros::NodeHandle`. You can think of the `ros::NodeHandle` as a "handle" for the
ROS node. All you need to know at this point is that it acts as the main access point for a lot of the ROS functionality, such as creating ROS publishers and subscribers.

Create a `ros::NodeHandle` by adding the following line in the main function:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh; // <-- Add this line

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```


#### 2. Create the `ros::Publisher`

After creating the `ros::NodeHandle`, we can now create a `ros::Publisher` for the node. `ros::NodeHandle` has a function
`advertise` that creates a `ros::Publisher` and returns it. Before we create the `ros::Publisher` though, we need to tell
ROS what type of message we will be publishing. In this case, since we want to publish a number, we can use the built-in
`std_msgs::Int32` message type. We also need the name of a topic where the messages will be published, which we'll
set to `my_number`.

(Note: `std_msgs` is a library provided by ROS for creating messages for standard types.
[Read more here](http://wiki.ros.org/std_msgs).)

Add an include for the `std_msgs/Int32.h`:
```c++
#include <ros/ros.h>
#include <std_msgs/Int32.h> // <-- Add this line

#include <iostream>
...
```

Then call the `advertise` function on the `nh` node handle we just created, and store the result in a publisher:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh;

  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1); // <-- Add this line
  // nh.advertise<TYPE>(TOPIC_NAME, QUEUE_SIZE)

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```

Notice that we specify the type of the message, `std_msgs::Int32`, as a **template argument** to the `advertise` function.
Don't worry about the details of how that works for now, we'll cover templates in a later week during general software
training. We specify the topic name as the first argument, and the queue size in the second argument. We have a queue size
of 1 set, meaning that we only want to publish the newest message if. Don't worry about the queue size for now.

#### 3. Create the message to be published

Now that we've got a publisher, let's publish a message. To do that, we first need to create the message which we want to
publish in a variable.

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh;

  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);
  
  std_msgs::Int32 message; // <-- Add this line

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```

After creating the message variable, we need to assign the message some data. To find out what **fields** are in a message,
we can make use of Clion's handy autocomplete. Type `message.` in the line below, and Clion should show the different fields
of a particular variable. In this case, we see that the `std_msgs::Int32` type has one field called `data`, which has a type
of `int`. Set the variable to any number you like:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh;

  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);
  
  std_msgs::Int32 message;
  message.data = 13; // <-- Add this line

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```

#### 4. Call the `publish()` function for the `ros::Publisher`

To actually publish the message, we call `integer_pub.publish` and pass the `std_msgs::Int32` message we just created
as an argument:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh;

  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);
  
  std_msgs::Int32 message;
  message.data = 13;            // <-- Add these lines
  integer_pub.publish(message); // <--

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```

One problem right now though is that the publisher needs to wait for the subscriber to connect first before publishing,
otherwise the message will be published before the subscriber will see it. A simple, though not the best, fix we can do
is to add in a `ros::Duration(1).sleep()` to make the program sleep for one second before publishing:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh;

  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);

  ros::Duration(1).sleep() // <-- Add in a sleep here to give time to subscribers to connect.
  
  std_msgs::Int32 message;
  message.data = 13;
  integer_pub.publish(message);

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```

Compile again with `catkin_make`, then run the node with `rosrun`. A message should be published to the `my_number` topic
with the number you sent. What command can you run to see the message that is published?
[Hint](#spoiler "rostopic echo") [Answer](#spoiler "rostopic  echo  /my_number").

Make sure you run the command before you launch the `week2_publisher` node. You should see something like this.
```
data: 13
---
```

### Writing a simple subscriber
Now that we've written a publisher, let's write a subscriber. Start off by setting up the node with `ros::init` in the 
[code/igvc_training_exercises/src/week2/subscriber.cpp](../igvc_training_exercises/src/week2/subscriber.cpp) file, but
with a node name of `subscriber` instead. Refer back to the
[code/igvc_training_exercises/src/week2/publisher.cpp](../igvc_training_exercises/src/week2/publisher.cpp) file
or above if you need to.
[Answer](#spoiler 'ros::init(argc, argv, "subscriber")'). Don't forget to add `#include <ros/ros.h>` so that you can use
the ros functions. There are again four things that we need to do:

#### 1. Create a `ros::NodeHandle`

Since we need to do ROS things, ie. create a Subscriber, we need a `ros::NodeHandle` also. Create one like before
again. [Answer](#spoiler 'ros::NodeHandle nh')

#### 2. Create a `ros::Subscriber`

Now, we need to create a `ros::Subscriber`. Similar to how we created a `ros::Publisher`, we can create a `ros::Subscriber`
by calling the `subscribe` function on the `ros::NodeHandle` like so:
```c++
ros::Subscriber integer_sub = nh.subscribe("my_number", 1, integerCallback);
// nh.subscribe(TOPIC_NAME, QUEUE_SIZE, CALLBACK_FUNCTION)
```

#### 3. Create a callback function

One thing different about writing a `ros::Subscriber` is that we need to write a **callback function**, which in here would
be the `integerCallback` function (which we haven't written yet, so Clion gives us an error).

##### ROS Subscribers and callbacks
What is a **callback function**? For a ROS subscriber **callback function** is a function that is called when a new message
is received. For example, in the example above, we've told ROS that we want the `integerCallback` function to be called
whenever we receive a message. So, if the `integerCallback` function was something like below

```c++
void integerCallback(std_msgs::Int32 message)
{
  std::cout << "I received a message: " << message.data << std::endl;
}
```

then you can understand it as if the ROS library was calling your **callback function** with the message that was received:

```c++
integerCallback(new_message) // You can imagine that this is happening somewhere.
```

##### Creating the callback function and ROS_INFO_STREAM
Let's create the callback function. Add this function above the `main` function:

```c++
void integerCallback(std_msgs::Int32 message)
{
  ROS_INFO_STREAM("I received a message: " << message.data);
}
```

One thing to notice is that instead of using `std::cout`, we are using `ROS_INFO_STREAM` here. `ROS_INFO_STREAM` is a
convenient method that ROS provides us for logging things. Usually, it is better to use `ROS_INFO_STREAM` than just
`std::cout`, because `ROS_INFO_STREAM` also includes a timestamp.

Try compiling with `catkin_make` and testing it out. Run the subscriber node first with rosrun
([Answer](#spoiler "rosrun  igvc_training_exercises  week2_subscriber")), then running the publisher node as before.


And...... nothing gets printed. What's happening?

#### 4. `ros::spin()`
The reason why nothing is happening is because the subscriber node doesn't have the `ros::spin` line that we put in the
publisher node.

Besides stopping the program from exiting until we hit Ctrl-C, calling `ros::spin` also hands control of the program to
ROS, so that ROS can do all the behind-the-scenes work. In this case, this would be calling the respective callback
functions when a new message comes in.

Let's add the missing `ros::spin` function right after we call `nh.subscribe`, and then recompile. Now, when you run the
subscriber and publisher, you should see the subscriber print out the message correctly:

```
[ INFO] [1565078804.617569525]: I received a message: 13
```

If you're not seeing the message print correctly, make sure that you run the subscriber node first, and then the
publisher node afterwards, otherwise the publisher node might publish before the subscriber node has a chance to receive
the message.

## Exercises
<details>
  <summary>1. A publisher that prints a sequence of numbers</summary>
  
  Now that you've written a simple publisher and subscriber, instead of publishing just a single number, publish the
  numbers from 0 to 99 in [exercises/loop_publisher.cpp](../igvc_training_exercises/src/week2/exercises/loop_publisher.cpp)
  on the same `my_number` topic. To do this, you will need to use a for loop. In case you forgot how to write a for loop:

  ```
  for (int i = 0; i < 100; i++)
  {
  }
  ```
</details>

<details>
  <summary>2. A subscriber for <code>std_msgs::String</code></summary>
  
  Write a subscriber that subscribes to the `/warning` topic with the `std_msgs::String` message type, and print out the
  contents of the message received in the callback function using `ROS_WARN_STREAM` instead of `ROS_INFO_STREAM`.
  
  Remember to `#include <std_msgs/String.h>` to be able to use that type.
  
  How can you test that your subscriber is working? 
  [Hint](#spoiler 'Use the "rostopic pub" command. Tab-completion works here, so keep pressing tab.'),
  [Answer](#spoiler 'rostopic pub /my_string std_msgs/String "data: \'Hello World!\'"').
  
</details>

<details>
  <summary>3. A subscriber that publishes if the received number is even</summary>
  
  Write a subscriber that subscribes to the same `my_number` topic as before in the
  [exercises/even_publisher.cpp](../igvc_training_exercises/src/week2/exercises/even_publisher.cpp).
  Instead of just printing it out though, this time have the node check if the number even. If the number is even,
  have the node publish the received number to a new topic `even_number`, otherwise do nothing.
  
  This will require you to make the `ros::Publisher` a **global variable** instead of a **local variable** inside of the `main`
  function, so that you can use the publisher variable inside the callback.
  
  Instead of defining the `ros::Publisher` as a **local variable** inside of the main function, ie.
  ```c++
  int main(int argc, char** argv)
  {
    ...
    ros::Publisher even_publisher = nh.advertise<std_msgs::Int32>("even_number", 1);
  }
  ```
  
  Define it as a **global variable** right below the includes, ie.
  
  ```c++
  ros::Publisher g_even_publisher;
  int main(int argc, char** argv)
  {
    ...
    g_even_publisher = nh.advertise<std_msgs::Int32>("even_number", 1);
  }
  ```
  
  Notice that we add the `g_` prefix to the variable name. This is to follow the ROS style guide, so that we can easily
  tell which variables are **global variables**.
  
  How can you tell if a number is even? 
  [Hint](#spoiler 'The % (modulo) operator returns the remainder after division of one number by another.'),
  [Answer](#spoiler 'message.data % 2 == 0').
  
  How do you publish to the `even_number` topic to test the node?
  [Hint](#spoiler 'rostopic pub /even_number <tab>')
  [Answer](#spoiler 'rostopic pub std_msgs/Int32 "data: 8')
  
  How do you echo the `even_number` topic to test that even numbers are being published?
  [Answer](#spoiler 'rostopic echo /even_number')
  
</details>

<details>
  <summary>4. A "safety" node for the simulator</summary>
  
  Write a node that's actually (somewhat) useful: A "safety" node that takes in `geometry_msgs/Twist` commands,
  and if the linear speed is too high, send a command with all 0s instead telling the robot to stop.
  
  Write this node in
  [exercises/safety_node.cpp](../igvc_training_exercises/src/week2/exercises/safety_node.cpp).
  
  We want to stop the robot if the **speed** is above a certain limit. We can do this easily by taking the absolute
  value of the velocity with the `std::abs` function.
  
  [Hint](#spoiler 'Have the node read from a topic like "/oswin/velocity_unsafe". You can then either rostopic pub
  or use the teleop_twist_keyboard command with the topic set to "/oswin/velocity_unsafe" to test it out')
</details>

## Summary
And that's it for this week!

This week, we learnt about:
- [Writing a ROS node](#hello-world-with-ros-and-c)
    + Using `ros::init(argc, argv, "NODE NAME HERE");` to create a ROS Node
    + `ros::spin()` so that nodes don't exit
- [Writing a ROS publisher](#writing-a-simple-publisher)
    + Creating a `ros::NodeHandle` to allow Nodes to do ROS things
    + Using `nh.advertise` to create a publisher
    + Using the `.publish` method on a publisher to publish a message on a topic
- [Writing a ROS subscriber](#writing-a-simple-subscriber)
    + Using `nh.subscribe` to create a subscriber
    + Creating a **callback function** to receive messages from the subscribed topic
    + `ros::spin()` as a "behind-the-scenes" method that calls the callbacks you defined

And hopefully we've had some practice with writing a few nodes, publishers and subscribers.

[Next week](week3.md), we'll learn about launch files and PID, and get to (properly) play with the
simulator.
