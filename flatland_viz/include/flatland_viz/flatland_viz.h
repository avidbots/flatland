/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   flatland_viz.h
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

#ifndef FLATLAND_VIZ_FLATLAND_VIZ_H
#define FLATLAND_VIZ_FLATLAND_VIZ_H

#include <QMainWindow>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <string>

#include "flatland_msgs/msg/debug_topic_list.hpp"

class QAction;
class QActionGroup;
class QToolBar;

namespace rviz_common
{
class Display;
class RenderPanel;
class Tool;
class VisualizationManager;
}  // namespace rviz_common

class FlatlandViz : public QMainWindow
{
  Q_OBJECT

public:
  /**
   * @brief Construct the rviz render panel, tools and displays, and subscribe to
   * the debug topic list
   *
   * @param rviz_ros_node The ros node used by rviz and flatland_viz
   * @param parent The parent widget
   */
  explicit FlatlandViz(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node,
    QWidget * parent = nullptr);

  ~FlatlandViz() override;

  /**
   * @brief Receive a new DebugTopicList msg and add/remove displays as required
   *
   * @param msg The DebugTopicList message
   */
  void ReceiveDebugTopics(const flatland_msgs::msg::DebugTopicList::SharedPtr msg);

private:
  void addTool(rviz_common::Tool * tool);
  void indicateToolIsCurrent(rviz_common::Tool * tool);
  void onToolbarActionTriggered(QAction * action);

  rviz_common::RenderPanel * render_panel_;
  rviz_common::VisualizationManager * manager_;

  QToolBar * toolbar_;
  QActionGroup * toolbar_actions_;
  std::map<QAction *, rviz_common::Tool *> action_to_tool_map_;
  std::map<rviz_common::Tool *, QAction *> tool_to_action_map_;

  std::map<std::string, rviz_common::Display *> debug_displays_;
  rclcpp::Subscription<flatland_msgs::msg::DebugTopicList>::SharedPtr debug_topic_subscriber_;
};

#endif  // FLATLAND_VIZ_FLATLAND_VIZ_H
