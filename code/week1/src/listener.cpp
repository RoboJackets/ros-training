#include "ros/ros.h"
#include "std_msgs/String.h"

// Creates callback to listen to a topic of type string
void chatterCallback(std_msgs::String msg)
{
  // prints outs to the terminal what in the message
  ROS_INFO("I heard %s", msg.data.c_str());
}


int main(int argc, char **argv)
{
  // sets up the node in the ros system, allows it to publish and subscribe
  ros::init(argc, argv, "listener");

  // Create a NodeHandle, This is the object used to interact with roscore (set up publishers and subscribers)
  ros::NodeHandle nh;

  // Creates a subscriber object called sub, it listens to topic chatter with a queue size of 1000, and tells ros what method it should call with the data chatterCallback method.
  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  // tells ROS to handle callbacks indefinitely
  ros::spin();

  // return zero to say that the program ran successfully
  return 0;
}
