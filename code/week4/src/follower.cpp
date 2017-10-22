#include <ros/ros.h>
#include <hal_msgs/speed_pair.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

ros::Publisher motor_pub;

void cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
  hal_msgs::speed_pair speed;
  speed.left_velocity = 1.0;
  speed.right_velocity = 1.0;
  int y_count = 0;
  for(pcl::PointXYZ point : msg->points) {
    ROS_INFO_STREAM("x = " << point.x << " y = " << point.y);
    if(pow(point.x, 2) + pow(point.y, 2) < 1) {
      speed.right_velocity = 0;
      speed.left_velocity = 0;
    }
    if(point.y > 0) {
      y_count++;
    } else {
      y_count--;
    }
  }
  if(abs(y_count) > 10) {
    if(y_count < 0) {
      speed.right_velocity = 1.0;
      speed.left_velocity = -1.0;
    } else {
      speed.right_velocity = -1.0;
      speed.left_velocity = 1.0;
    }
  }
  ROS_INFO_STREAM(speed.right_velocity << " " << speed.left_velocity << "\n\n");
  motor_pub.publish(speed);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "follower");

  ros::NodeHandle nh;

  ros::Subscriber point_cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/scan/pointcloud", 1, cloud_callback);

  motor_pub = nh.advertise<hal_msgs::speed_pair>("/motors", 1);

  ros::spin();
}
