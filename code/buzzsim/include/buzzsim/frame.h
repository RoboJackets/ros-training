#ifndef SRC_FRAME_H
#define SRC_FRAME_H

#include <memory>

#include <ros/ros.h>

#include <QFrame>

#include <buzzsim/turtle.h>

#include <QLabel>
#include <QGridLayout>
#include <QTimer>
#include <QElapsedTimer>

#include <buzzsim/world.h>

class BuzzsimFrame : public QFrame
{
  Q_OBJECT
 public:
  explicit BuzzsimFrame(QWidget* parent = nullptr,  Qt::WindowFlags f = nullptr);
  ~BuzzsimFrame() override;

 protected:
  void paintEvent([[maybe_unused]] QPaintEvent *) override;

 private slots:
  void onUpdate();

 private:
  std::vector<QImage> getTurtleImages() const;

  QTimer* update_timer_;
  std::unique_ptr<turtle::Turtle> turtle_;
  World world_;

  ros::WallTime last_update_;
};

#endif // SRC_BUZZSIM_FRAME_H
