/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   flatland_viz.cpp
 * @brief  Manages the librviz viewport for flatland
 * @author Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

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
    const flatland_msgs::DebugTopicList::ConstPtr& msg) {
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
