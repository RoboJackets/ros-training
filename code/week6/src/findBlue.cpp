#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

ros::Publisher image_publisher;

cv::Mat findBlue(const cv::Mat& frameBGR) {
  const cv::Scalar blue_low{78, 50, 70};
  const cv::Scalar blue_high{138, 255, 255};

  cv::Mat frameHSV;
  cvtColor(frameBGR, frameHSV, CV_BGR2HSV);
  cv::Mat output_blue = cv::Mat::zeros(frameHSV.size().height, frameHSV.size().width, CV_8U);
  inRange(frameHSV, blue_low, blue_high, output_blue);
  return output_blue;
}

void image_callback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat result_image = findBlue(cv_ptr->image);
  cv_ptr->image = result_image;
  cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;

  image_publisher.publish(cv_ptr->toImageMsg());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "findBlue");

  ros::NodeHandle nh;

  ros::Subscriber image_subscriber = nh.subscribe("/hal/usb_cam_center/image_raw", 1, image_callback);

  image_publisher = nh.advertise<sensor_msgs::Image>("image_modifed", 1);

  ros::spin();

}
