#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <math.h>

cv_bridge::CvImagePtr cv_ptr;
ros::Publisher debug_img_pub;
ros::Publisher start_pub;
bool prev_start_bool = false;
std_msgs::Bool prev_start_msg;
ros::Time last_red_time;

using namespace std;

int min_blob_area, laplacian_threshold_min, laplacian_threshold_max;

cv::Mat kernel(int x, int y) {
    return cv::getStructuringElement(cv::MORPH_RECT, cv::Size(x, y));
}

cv::Mat cutSmall(const cv::Mat& color_edges, int size_min) {
    cv::Mat contours_color(color_edges.size(), CV_8UC1, cv::Scalar::all(0));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(color_edges, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    for (u_long i = 0; i < contours.size(); i++)
        if (size_min < cv::contourArea(contours[i], false))
            cv::drawContours(contours_color, contours, i, cv::Scalar(255), CV_FILLED, 8);

    return contours_color;
}

void img_callback(const sensor_msgs::ImageConstPtr &msg) {
    cv::Mat frame  = cv_bridge::toCvCopy(msg, "bgr8")->image;


    cv::Mat frame_gray, frame_blur, lapl, adaptive, true_lines, floodfill_blobs;
    cv::GaussianBlur(frame, frame_blur, cv::Size(5, 5), 0);
    cv::cvtColor(frame_blur, frame_gray, cv::COLOR_BGR2GRAY);

    cv::Laplacian(frame_gray, lapl, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);
    inRange(lapl, laplacian_threshold_min, laplacian_threshold_max, lapl);

    cv::rectangle(lapl, cv::Point(0, 0), cv::Point(lapl.cols,  lapl.rows/2), 0, CV_FILLED);
    cv::dilate(lapl, lapl, kernel(4, 4));
//    cv::morphologyEx(lapl, lapl, ORPH_CLOSE, kernel(3, 3));
    lapl = cutSmall(lapl, 1000);

    frame = frame.setTo(cv::Scalar(0, 255, 0), lapl);

    sensor_msgs::Image outmsg;
    cv_ptr->image = frame;
    cv_ptr->encoding = "bgr8";
    cv_ptr->toImageMsg(outmsg);
    debug_img_pub.publish(outmsg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "week2_line_detector");
    ros::NodeHandle pnh{"~"};

    pnh.getParam("laplacian_threshold_min", laplacian_threshold_min);
    pnh.getParam("laplacian_threshold_max", laplacian_threshold_max);

    ros::Subscriber img_sub = pnh.subscribe("/camera/image", 1, img_callback);
    debug_img_pub = pnh.advertise<sensor_msgs::Image>("/camera/debug_img", 1);

    ros::spin();
}