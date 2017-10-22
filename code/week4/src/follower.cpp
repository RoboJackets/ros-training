#include <ros/ros.h>
#include <hal_msgs/speed_pair.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

ros::Publisher motor_pub;

void cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
  hal_msgs::speed_pair speed;
  speed.left_velocity = 0;
  speed.right_velocity = 0;

}


int main(int argc, char** argv) {

}
