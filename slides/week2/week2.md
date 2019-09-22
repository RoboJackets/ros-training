---
title: Week 2
---
# Week 2

---

## Recap of last week
- Setup VMs
- Learnt about ROS topics, messages, subscribers and publishers
- Learnt about the `rostopic` tool
- Wrote your own custom ROS message

---

## What are we doing today?
- Writing a ROS Publisher and Subscriber in C++
- Exercises

---

## Writing a ROS Subscriber and Publisher in C++
- We've looked at how we can publish messages, now it's time to do it in C++.

- Before we do that though, let's write a node.

---

### Hello World with ROS and C++
- Let's start by writing Hello World with ROS and C++.
- in [publisher.cpp](../igvc_training_exercises/src/week2/publisher.cpp):

```C++
#include <iostream>

int main(int argc, char** argv)
{
  std::cout << "Hello World!" << std::endl;
}
```

----

- We've already compiled this when compiling last week
- Recap:
    ```bash
    cd catkin_ws # cd to where your catkin workspace is located
    catkin_make
    ```

----

- Now, try running the executable with `rosrun`
    - ROS package: `igvc_training_exercises`
    - Executable: `week2_publisher`.

- How do you run `week2_publisher`?

<pre><code class="bash">rosrun igvc_training_exercises week2_publisher</code></pre> <!-- .element: class="fragment" data-fragment-index="1" --> 

----

- Verify that Hello World correctly prints out:
```
Hello World
```

----

- Although we have a Hello World executable, this isn't a node yet. To make this a proper ROS node, we need to initialize
it.

----

- Start by adding an `#include` for the ROS headers at the top of the file.
- We'll be covering what this does in general software training later.

----

```c++
#include <ros/ros.h> // <-- Add this line

#include <iostream>
...
```

----

- Next, add `ros::init(argc, argv, "week2")` above the `std::cout`:
    ```c++
    int main(int argc, char** argv)
    {
      ros::init(argc, argv, "week2"); // <-- Add this line
    
      std::cout << "Hello World!" << std::endl;
    }
    ```

---

```c++
ros::init(argc, argv, "NODE_NAME")
```

- initializes a ROS node
- last argument in the function is the name of the node

----

- Recompile with `catkin_make` and run the node again to verify that it still works
    - Make sure you have `roscore` running in a separate window, as ROS needs `roscore` to work.
- You should still see `Hello World` being printed out
- However, we can't actually tell that it's working, because the program is exiting immediately

----

- To make the program wait, add `ros::spin()`

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");

  std::cout << "Hello World!" << std::endl;
  ros::spin(); // <-- Add this line
}
```

- For now, don't worry about what `ros::spin()` does. All you need to know is that it stops the program from exiting until
you do Ctrl-C to stop it.

----

- Compile the program again, and run it
    - The program now doesn't exit
- Open up a new terminal window and type in `rosnode list`
- You should see the `week2` node show up:
```
/rosout
/week2
```

---

### Writing a simple publisher
Now, let's write a simple publisher that publishes a number. To do that, we need to do four things:

----

#### 1. Create a `ros::NodeHandle`

- `ros::NodeHandle` as a "handle" for the ROS node
- Main access point for a lot of the ROS functionality
    - Creating ROS publishers and subscribers.

----

- Create a `ros::NodeHandle` by adding the following line in the main function:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh; // <-- Add this line

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```

----

#### 2. Create the `ros::Publisher`

Now we can create a `ros::Publisher` for the node.

----

- `ros::NodeHandle` has a function `advertise` that creates a `ros::Publisher`
- Need to specify the _type_ of the message
    - Number -> `std_msgs::Int32`.
- Also need a topic name
    - `my_number`

`std_msgs` is a library provided by ROS for creating messages for standard types

----

Add an include for the `std_msgs/Int32.h`:
```c++
#include <ros/ros.h>
#include <std_msgs/Int32.h> // <-- Add this line

#include <iostream>
...
```

----

Then call the function on the `nh` node handle we just created, and store the result in a publisher:

```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2");
  ros::NodeHandle nh;

  // Add the line below
  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);

  std::cout << "Hello World!" << std::endl;
  ros::spin();
}
```

----

`nh.advertise<TYPE>(TOPIC_NAME, QUEUE_SIZE)`

- Specify the type of the message, `std_msgs::Int32`, as a **template argument** to the `advertise` function
- We'll cover templates in a later week during general software training
- Queue size of 1 => only want to publish the newest message
    - Don't worry about the queue size
    
----

#### 3. Create the message to be published

Now that we've got a publisher, let's publish a message.
- Create the message which we want to publish in a variable.

----

- Since the type of the message is `std_msgs::Int32`, we make a variable of the same type

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

----

- Need to assign the message some data
- To find out what **fields** are in a message, we can make use of Clion's handy autocomplete
- Type `message.` in the line below
    - clion's autocomplete should show fields
    - one field `data` of type `int`

----

- set the `message.data` to any number you like:

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

----

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

----

- One problem: publisher needs to wait for the subscriber to connect first before publishing
    - Otherwise message will be published before the subscriber will see it
- Add `ros::Duration(1).sleep()` to sleep for one second

```c++
  ...
  ros::Publisher integer_pub = nh.advertise<std_msgs::Int32>("my_number", 1);

  ros::Duration(1).sleep() // <-- Add in a sleep here to give time to subscribers to connect.
  
  std_msgs::Int32 message;
  message.data = 13;
  integer_pub.publish(message);
  ...
```

----

#### Checking that the publisher works

- Compile with `catkin_make`, then run with `rosrun`
- How do you run the node? (The name of the executable is `week2_publisher`)
    <ul><li><pre><code>rosrun igvc_training_exercises week2_publisher</code></pre></li></ul><!-- .element: class="fragment" data-fragment-index="1" --> 
<li><p>A message should be published to the `my_number` topic</p></li> <!-- .element: class="fragment" data-fragment-index="2" --> 
<li><p>How do you see the message that the node published?</p></li> <!-- .element: class="fragment" data-fragment-index="2" --> 
    <ul><li><pre><code>rostopic echo /my_number</code></pre></li></ul> <!-- .element: class="fragment" data-fragment-index="3" --> 

----
 
Make sure you run the command before you launch the `week2` node. You should see something like this.
```
data: 13
```


---

### Writing a simple subscriber
Now that we've written a publisher, let's write a subscriber

----

#### 0. Blank main to ROS node

- Start off with [code/igvc_training_exercises/src/week2/subscriber.cpp](../igvc_training_exercises/src/week2/subscriber.cpp)
- Node name of `subscriber` instead
- Refer back to the
[code/igvc_training_exercises/src/week2/publisher.cpp](../igvc_training_exercises/src/week2/publisher.cpp) if you need to
- Don't forget `#include <ros/ros.h>`
- There are again _four_ things that we need to do:

----

#### 1. Create a `ros::NodeHandle`

Since we need to do ROS things, ie. create a Subscriber, we need a `ros::NodeHandle` also

- Create one like before again
- How do you create a `ros::NodeHandle`?
<ul><li><pre><code>ros::NodeHandle nh;</code></pre></li></ul> <!-- .element: class="fragment" data-fragment-index="1" --> 

----

#### 2. Create a `ros::Subscriber`

- Now, we need to create a `ros::Subscriber`.
    - Similar to `ros::Publisher`, we can create a `ros::Subscriber` by calling the `subscribe` function on
    `ros::NodeHandle`:
    ```c++
    ros::Subscriber integer_sub = nh.subscribe("my_number", 1, integerCallback);
    ```
<li><pre><code>nh.subscribe(TOPIC_NAME, QUEUE_SIZE, CALLBACK_FUNCTION);</code></pre></li> <!-- .element: class="fragment" data-fragment-index="1" --> 

----

#### 3. Create a callback function

- One thing different is that we need to write a **callback function** (the `integerCallback` function)
- We haven't written yet, so Clion gives us an error

----

##### What is a callback function?

- In general, a **callback function** is a function which is passed into some other function
    - Called when some _event_ happens
- For ROS subscribers, the event is when a message is received
    - The **callback function** is called when the ROS subscriber received a new message
    
----

- For example, if the `integerCallback` function was something like below

```c++
void integerCallback(std_msgs::Int32 message)
{
  std::cout << "I received a message: " << message.data << std::endl;
}
```

- Understand it as the ROS library calling your **callback function** with the new message

```c++
integerCallback(new_message) // You can imagine that this is happening somewhere.
```

----

##### Creating the callback function and ROS_INFO_STREAM
- Let's create the callback function. Add this function above the `main` function:
    ```c++
    void integerCallback(std_msgs::Int32 message)
    {
      ROS_INFO_STREAM("I received a message: " << message.data);
    }
    ```

<pre><code class="c++">ROS_INFO_STREAM</code></pre>
<ul>
    <li>Convenient method that ROS provides us for logging things</li>
    <li>Usually better than `std::cout` beacuse also includes a timestamp</li>
</ul>
    
----

#### Testing the subscriber

- Try compiling with `catkin_make` and testing it out
- Run the subscriber node first with `rosrun`, then run the publisher node

And...... nothing gets printed. What's happening? <!-- .element: class="fragment" data-fragment-index="1" --> 

----

#### 4. `ros::spin()`
The subscriber node doesn't have the `ros::spin`

----

##### What does `ros::spin()` do

- Stops the program from exiting until we hit Ctrl-C
- Hands control of the program to ROS, so that ROS can do all the behind-the-scenes work
    - Allow ROS to call the **callback function** when a new message is received
    
----

- Add the missing `ros::spin` function right after we call `nh.subscribe`
- Recompile and run
    - Should see the subscriber print out the message correctly
    ```
    [ INFO] [1565078804.617569525]: I received a message: 13
    ```

----

If you're not seeing the message print correctly, make sure that you run the subscriber node first, and then the
publisher node afterwards, otherwise the publisher node might publish before the subscriber node has a chance to receive
the message.

---

## Your Turn

- Follow either the markdown version or the slides on the github repo
<ul><li><a href="https://github.com/RoboJackets/ros-training">github.com/RoboJackets/ros-training</a></li></ul>
- Show us the finished Publisher and Subscriber working
- If you're done with both, there are extra exercises in the markdown version

---

## Summary

----

- [Creating a ROS Node](#/3)
    + `ros::init(argc, argv, "NODE_NAME")`

----

- [Writing A ROS Publisher](#/7)
    + Need a `ros::NodeHandle`
    + `ros::Publisher publisher = nh.advertise<MESSAGE_TYPE>(TOPIC, QUEUE_SIZE)`
    + Create the message
    + `publisher.publish(message)`

----

- [Writing a ROS Subscriber](#/8)
    + Need a `ros::NodeHandle`
    + `ros::Subscriber subscriber = nh.subscribe(TOPIC, QUEUE_SIZE, CALLBACK_FUNCTION)`
        + A **Callback Function** is called when a new message is received
    + Create the **callback function**
    + `ros::spin()` to let ROS handle the messages

----

## Next Week
Next week we'll learn about **launch files** and **PID**, and get to (properly) play with the
simulator.

---

See you next week!
