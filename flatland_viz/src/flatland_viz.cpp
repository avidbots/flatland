#include <OgreColourValue.h>
#include <QColor>
#include <QGridLayout>
#include <QLabel>
#include <QSlider>
#include <QVBoxLayout>

#include <ros/ros.h>
#include <stdlib.h>

#include "rviz/display.h"
#include "rviz/render_panel.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"

#include "flatland_viz/flatland_viz.h"

FlatlandViz::FlatlandViz(FlatlandWindow* parent) : QWidget((QWidget*)parent) {
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  // main_layout->setSpacing(0);
  main_layout->setMargin(0);
  main_layout->addWidget(render_panel_);

  // Set the top-level layout for this FlatlandViz widget.
  setLayout(main_layout);

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();

  // Set view controller to top down
  manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
  render_panel_->setBackgroundColor(Ogre::ColourValue(0.2, 0.2, 0.2));

  // Create a Grid display.
  grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
  if (grid_ == nullptr) {
    ROS_FATAL("Grid failed to instantiate");
    exit(1);
  }

  // Configure the GridDisplay the way we like it.
  grid_->subProp("Line Style")->setValue("Lines");
  grid_->subProp("Color")->setValue(QColor(Qt::white));
  grid_->subProp("Cell Size")->setValue(1.0);
  grid_->subProp("Plane Cell Count")->setValue(100);
  grid_->subProp("Alpha")->setValue(0.1);

  // Subscribe to debug topics topic
  ros::NodeHandle n;
  debug_topic_subscriber_ = n.subscribe("/flatland_server/debug/topics", 0,
                                        &FlatlandViz::RecieveDebugTopics, this);
}

void FlatlandViz::RecieveDebugTopics(
    const flatland_server::DebugTopicList::ConstPtr& msg) {
  for (const auto& name : msg->topics) {
    if (debug_topics_.count(name) == 0) {
      // Insert the name into the topics set to mark that it's loaded
      debug_topics_.insert(name);

      // Create the marker display and set its topic
      auto markers = manager_->createDisplay(
          "rviz/MarkerArray", QString::fromLocal8Bit(name.c_str()), true);
      if (markers == nullptr) {
        ROS_FATAL("NarkerArray failed to instantiate");
        exit(1);
      }
      QString topic = QString::fromLocal8Bit(
          (std::string("/flatland_server/debug/") + name).c_str());
      markers->subProp("Marker Topic")->setValue(topic);
    }
  }
}

// Destructor.
FlatlandViz::~FlatlandViz() { delete manager_; }
