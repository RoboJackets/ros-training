#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

ros::Publisher image_publisher;

float kdata[] = {0, 1, 0, 1, -4, 1, 0, 1, 0};

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat grey_scale;
  cvtColor(cv_ptr->image,grey_scale,CV_RGB2GRAY);

  cv::Mat kernel(3,3,CV_32F, kdata);
  cv::Mat result;
  filter2D(grey_scale, result, CV_8U, kernel);
  cv_ptr->image = result;
  cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;

  image_publisher.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_processing");

  ros::NodeHandle nh;

  ros::NodeHandle pNh("~");

  std::vector<double> kernel_data;
  pNh.getParam("kernel", kernel_data);
  for(int i = 0; i < kernel_data.size(); i++) {
    kdata[i] = kernel_data.at(i);
  }

  ros::Subscriber image_subscriber = nh.subscribe("/usb_cam_center/image_raw", 1, image_callback);

  image_publisher = nh.advertise<sensor_msgs::Image>("image_modifed", 1);

  ros::spin();
}
