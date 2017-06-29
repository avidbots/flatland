#include <QApplication>
#include <ros/ros.h>
#include "flatland_viz/flatland_viz.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "flatland_viz", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  FlatlandViz* viz = new FlatlandViz();
  viz->show();

  app.exec();

  delete viz;
}