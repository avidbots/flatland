#include <ros/ros.h>
#include <QApplication>
#include "flatland_viz/flatland_window.h"

int main(int argc, char** argv) {
  if (!ros::isInitialized()) {
    ros::init(argc, argv, "flatland_viz", ros::init_options::AnonymousName);
  }

  QApplication app(argc, argv);

  FlatlandWindow* w = new FlatlandWindow();
  w->show();

  app.exec();

  delete w;
}