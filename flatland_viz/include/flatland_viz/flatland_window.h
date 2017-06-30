#include <ros/ros.h>
#include <QLabel>
#include <QMainWindow>
#include <QWidget>
#include "flatland_viz/flatland_viz.h"

class FlatlandWindow : public QMainWindow {
  Q_OBJECT
 public:
  FlatlandWindow(QWidget* parent = 0);

  QLabel* fps_label_;

 protected Q_SLOTS:
  void UpdateFps();

 private:
  FlatlandViz* viz_;
  int frame_count_;
  ros::WallTime last_fps_calc_time_;
};