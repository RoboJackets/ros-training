#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

ros::Publisher image_publisher;

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat grey_scale;
  cvtColor(cv_ptr->image,grey_scale,CV_RGB2GRAY);
  cv_ptr->image = grey_scale;
  cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;

  image_publisher.publish(cv_ptr->toImageMsg());
  // all of the image processing
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_processing");

  ros::NodeHandle nh;

   ros::Subscriber image_subscriber = nh.subscribe("/hal/usb_cam_center/image_raw", 1, image_callback);

  image_publisher = nh.advertise<sensor_msgs::Image>("image_modifed", 1);

  ros::spin();
}
