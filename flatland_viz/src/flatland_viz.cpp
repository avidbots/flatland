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

#include "flatland_viz/flatland_viz.h"

#include <QAction>
#include <QActionGroup>
#include <QColor>
#include <QCoreApplication>
#include <QScreen>
#include <QToolBar>
#include <algorithm>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_rendering/render_window.hpp>

FlatlandViz::FlatlandViz(
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget * parent)
: QMainWindow(parent)
{
  setWindowTitle("Flatland Viz");
  resize(screen()->availableSize() * 0.9);

  toolbar_ = addToolBar("Tools");
  toolbar_->setObjectName("Tools");
  toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toolbar_actions_ = new QActionGroup(this);
  connect(toolbar_actions_, &QActionGroup::triggered, this, &FlatlandViz::onToolbarActionTriggered);

  render_panel_ = new rviz_common::RenderPanel(this);
  setCentralWidget(render_panel_);

  // In rviz2 the render window must be initialized before the VisualizationManager is built.
  // Flush pending Qt window-system events first, or Ogre's GL context can't bind to the window.
  QCoreApplication::processEvents();
  render_panel_->getRenderWindow()->initialize();
  rclcpp::Node::SharedPtr node = rviz_ros_node.lock()->get_raw_node();
  manager_ =
    new rviz_common::VisualizationManager(render_panel_, rviz_ros_node, nullptr, node->get_clock());
  render_panel_->initialize(manager_);
  manager_->initialize();

  // Replace rviz2's default tools with the flatland toolset; the first one is the default tool
  rviz_common::ToolManager * tool_man = manager_->getToolManager();
  tool_man->removeAll();
  connect(tool_man, &rviz_common::ToolManager::toolAdded, this, &FlatlandViz::addTool);
  connect(
    tool_man, &rviz_common::ToolManager::toolChanged, this, &FlatlandViz::indicateToolIsCurrent);
  tool_man->addTool("rviz_default_plugins/MoveCamera");
  tool_man->addTool("flatland_rviz_plugins/Interact");
  tool_man->addTool("flatland_rviz_plugins/SpawnModel");
  tool_man->addTool("flatland_rviz_plugins/TogglePause");
  tool_man->addTool("flatland_rviz_plugins/ChangeRate");

  manager_->setFixedFrame("map");
  manager_->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/TopDownOrtho");

  rviz_common::Display * grid =
    manager_->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
  grid->subProp("Line Style")->setValue("Lines");
  grid->subProp("Color")->setValue(QColor(Qt::white));
  grid->subProp("Cell Size")->setValue(1.0);
  grid->subProp("Plane Cell Count")->setValue(100);
  grid->subProp("Alpha")->setValue(0.1);

  // Enabled/disabled by the flatland_rviz_plugins/Interact tool
  rviz_common::Display * interactive_markers =
    manager_->createDisplay("rviz_default_plugins/InteractiveMarkers", "Move Objects", false);
  interactive_markers->subProp("Interactive Markers Namespace")
    ->setValue("interactive_model_markers");

  // Callbacks are serviced on the Qt thread by the VisualizationManager's executor
  using std::placeholders::_1;
  debug_topic_subscriber_ = node->create_subscription<flatland_msgs::msg::DebugTopicList>(
    "topics", rclcpp::QoS(1).transient_local(),
    std::bind(&FlatlandViz::RecieveDebugTopics, this, _1));

  manager_->startUpdate();
}

FlatlandViz::~FlatlandViz()
{
  debug_topic_subscriber_.reset();
  delete manager_;
}

void FlatlandViz::addTool(rviz_common::Tool * tool)
{
  QAction * action = new QAction(tool->getName(), toolbar_actions_);
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
  action->setCheckable(true);
  toolbar_->addAction(action);
  action_to_tool_map_[action] = tool;
  tool_to_action_map_[tool] = action;
}

void FlatlandViz::indicateToolIsCurrent(rviz_common::Tool * tool)
{
  auto it = tool_to_action_map_.find(tool);
  if (it != tool_to_action_map_.end()) {
    it->second->setChecked(true);
  }
}

void FlatlandViz::onToolbarActionTriggered(QAction * action)
{
  auto it = action_to_tool_map_.find(action);
  if (it != action_to_tool_map_.end()) {
    manager_->getToolManager()->setCurrentTool(it->second);
  }
}

void FlatlandViz::RecieveDebugTopics(const flatland_msgs::msg::DebugTopicList::SharedPtr msg)
{
  const std::vector<std::string> & topics = msg->topics;

  // remove displays for deleted topics
  for (auto it = debug_displays_.begin(); it != debug_displays_.end();) {
    if (std::find(topics.begin(), topics.end(), it->first) == topics.end()) {
      delete it->second;
      it = debug_displays_.erase(it);
    } else {
      ++it;
    }
  }

  // add displays for new topics
  for (const auto & topic : topics) {
    if (debug_displays_.count(topic) == 0) {
      rviz_common::Display * display = manager_->createDisplay(
        "rviz_default_plugins/MarkerArray", QString::fromStdString(topic), true);
      rviz_common::properties::Property * topic_prop = display->subProp("Topic");
      // flatland_server publishes debug markers latched (transient local)
      topic_prop->subProp("Durability Policy")->setValue("Transient Local");
      topic_prop->setValue(QString::fromStdString(topic));
      debug_displays_[topic] = display;
    }
  }
}
