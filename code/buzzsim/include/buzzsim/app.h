#ifndef SRC_APP_H
#define SRC_APP_H

#include <QtWidgets>
#include <ros/spinner.h>

class BuzzsimApp : public QApplication
{
 public:
  BuzzsimApp(int& argc, char** argv);
  int exec();

 private:
  std::unique_ptr<ros::AsyncSpinner> spinner_;
};

#endif //SRC_APP_H
