#include <ros/ros.h>
#include <hal_msgs/speed_pair.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

ros::Publisher motor_pub;
double desired_speed;
double last_distance = 10;
double last_speed = 0;

void cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
  hal_msgs::speed_pair speed;
  speed.left_velocity = 3 * desired_speed;
  speed.right_velocity = 3 * desired_speed;
  int y_count = 0;
  double min_dist = 0;
  for(pcl::PointXYZ point : msg->points) {
    double distance = sqrt(pow(point.x, 2) + pow(point.y, 2));
    if(distance < 1.0) {
      speed.right_velocity = 0;
      speed.left_velocity = 0;
    }
    if(min_dist == 0 || distance < min_dist) {
      min_dist = distance;
    }
    if(point.y > 0) {
      y_count++;
    } else {
      y_count--;
    }
  }
  if(abs(y_count) > 10) {
    if(y_count < 0) {
      speed.right_velocity = desired_speed;
      speed.left_velocity = -desired_speed;
    } else {
      speed.right_velocity = -desired_speed;
      speed.left_velocity = desired_speed;
    }
  } else if(last_distance < min_dist) {
    speed.right_velocity = last_speed + desired_speed;
    speed.left_velocity = last_speed + desired_speed;
  }
  last_distance = min_dist;
  last_speed = fabs(speed.right_velocity);
  motor_pub.publish(speed);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "follower");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  pNh.getParam("desired_speed", desired_speed);

  ros::Subscriber point_cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/scan/pointcloud", 1, cloud_callback);

  motor_pub = nh.advertise<hal_msgs::speed_pair>("/motors", 1);

  ros::spin();
}
