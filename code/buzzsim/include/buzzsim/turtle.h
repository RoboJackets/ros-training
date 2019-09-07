#ifndef SRC_TURTLE_H
#define SRC_TURTLE_H

#include <mutex>
#include <random>

#include <QImage>
#include <QPainter>

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <tf/transform_broadcaster.h>

#include <buzzsim/motion.h>
#include <buzzsim/sensors/lidar.h>
#include <buzzsim/sensors/lidar_painter.h>
#include <buzzsim/obstacle.h>

namespace turtle
{
class Turtle
{
public:
  struct PublishOptions
  {
    bool imu;
    bool pose;
    bool lidar;

    [[nodiscard]] bool hasPublisher() const;
  };

  struct ImuStdDevs
  {
    double accelerometer{};
    double gyro{};
    double magnetometer{};
  };

  struct Options
  {
    std::string name;
    QImage turtle_image;
    motion::State state{};
    motion::Limits limits{};
    ImuStdDevs imu_std_devs_{};
    Lidar::Options lidar_options_{};
    LidarPainter::Options lidar_painter_options_{};
    PublishOptions publish_options{};
  };

  struct IMUNoise
  {
    std::mt19937 gen_{ std::random_device{}() };
    ImuStdDevs imu_std_devs_{};
    std::normal_distribution<> acccelerometer_noise_{};
    std::normal_distribution<> gyro_noise_{};
    std::normal_distribution<> magnetometer_noise_{};

    IMUNoise(const ImuStdDevs& sensor_std_devs);
    double accelerometerNoise();
    double gyroNoise();
    double magnetometerNoise();
  };

  Turtle(const Options& options, const std::vector<Obstacle>* obstacles);
  void paint(QPainter& painter);
  void updatePose();

  motion::State state_;
  motion::Acceleration acceleration_;
  motion::Limits limits_;
  motion::Twist setpoint_{};

  PublishOptions publish_options_{};

private:
  void setupPubSub();
  void updateRotatedImage();
  void velocityCallback(const geometry_msgs::TwistConstPtr& twist);
  void publishCallback([[maybe_unused]] const ros::TimerEvent&);

  void publishPose();
  void publishTransform();
  void publishIMU();
  void publishLidar();

  void drawLidar(QPainter& painter, int width, int height);

  motion::Acceleration getAcceleration();

  std::string name_;

  std::mutex setpoint_mutex_;

  const std::vector<Obstacle>* obstacles_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  ros::NodeHandle nh_;
  ros::Subscriber velocity_sub_;

  ros::Publisher imu_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher lidar_pub_;

  ros::Timer publish_timer_;

  tf::TransformBroadcaster broadcaster_;
  IMUNoise imu_noise_;
  motion::State last_state_{};
  ros::Time last_time_{};

  Lidar lidar_;
  LidarPainter lidar_painter_;
  pcl::PointCloud<pcl::PointXYZ> last_pointcloud_;
  std::mutex pointcloud_mutex_;
};

std::ostream& operator<<(std::ostream& os, const Turtle::Options& options);
}  // namespace turtle
#endif  // SRC_TURTLE_H
