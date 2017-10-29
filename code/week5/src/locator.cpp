#include <ros/ros.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

void position_callback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    int index = 0;
    geometry_msgs::Pose pose;
    for(std::string current_model : msg->name) {
        if(current_model == "ball") {
          pose = msg->pose[index];
          tf::TransformBroadcaster br;
          tf::StampedTransform transform;
          transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
          transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
          transform.stamp_ = ros::Time::now();
          transform.child_frame_id_ = "/ball";
          transform.frame_id_ = "/base_link";
          br.sendTransform(transform);
          ROS_INFO("transform sent");
        }
        index++;
    }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "locator");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<gazebo_msgs::ModelStates>("hal/gazebo/model_states", 1, position_callback);

  ros::spin();
}
