#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
  // registers node with roscore, makes it so it can publish and subscribe with other nodes
  ros::init(argc, argv, "talker");

  // Create a NodeHandle, This is the object used to interact with roscore (set up publishers and subscribers)
  ros::NodeHandle nh;

  // Creates a publisher, object name chatter_pub, topic name chatter, type String, queue size 1000
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  // Creates a rate, this object will be used to later to slow down the node when publishing.
  ros::Rate loop_rate(10);

  // creates counter, counts the message number
  int count = 0;

  // this will keep running until is loses connection to ros, always use this and do not worry about it
  while(ros::ok()) {

    // Creates a String message
    std_msgs::String msg;

    // Create a string stream, essentially a string that I can add more characters to
    std::stringstream ss;

    // adds the string hello world and the current count to the string stream
    ss << "hello world " << count;

    // Sets the String message message to the string that I have created in the above line
    msg.data = ss.str();

    // prints out to the terminal the string I put in the message
    ROS_INFO("%s", msg.data.c_str());

    // publishes the message into the ros system using the chatter publisher, therefore it will be published on the chatter topic with a queue size of 1000
    chatter_pub.publish(msg);

    // gives ros control to do networking things
    ros::spinOnce();

    // Waits until 0.1 s since last time.
    loop_rate.sleep();

    // increments count
    count++;
  }
  // return zero to say that the program ran successfully
  return 0;
}
