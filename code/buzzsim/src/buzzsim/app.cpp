#include <buzzsim/app.h>
#include <buzzsim/frame.h>

BuzzsimApp::BuzzsimApp(int& argc, char** argv) : QApplication(argc, argv)
{
  ros::init(argc, argv, "buzzsimsim", ros::init_options::NoSigintHandler);
  spinner_ = std::make_unique<ros::AsyncSpinner>(1);
  spinner_->start();
}

int BuzzsimApp::exec()
{
  BuzzsimFrame frame;
  frame.show();

  return QApplication::exec();
}
