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

#include <ros/ros.h>
#include <QAction>
#include <QActionGroup>
#include <QList>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QToolBar>
#include <QToolButton>
#include <QWidget>
#include <map>
#include <set>

#include "flatland_msgs/DebugTopicList.h"
#include "rviz/config.h"
#include "rviz/panel.h"
#include "rviz/properties/property_tree_widget.h"
#include "rviz/tool.h"
#include "rviz/tool_manager.h"
#include "rviz/window_manager_interface.h"

class QSplashScreen;
class QAction;
class QActionGroup;
class QTimer;
class QDockWidget;
class QLabel;
class QToolButton;

namespace rviz {
class PanelFactory;
class Tool;
class Display;
class RenderPanel;
class VisualizationManager;
class WidgetGeometryChangeDetector;
}

class FlatlandWindow;

class FlatlandViz : public QWidget {
  Q_OBJECT public :
      /**
        * @brief Construct FlatlandViz and subscribe to debug topic list
        *
        * @param parent The parent widget
        */
      FlatlandViz(FlatlandWindow* parent = 0);

  /**
   * @brief Recieve a new DebugTopicList msg and add any new displays required
   *
   * @param msg The DebugTopicList message
   */
  void RecieveDebugTopics(const flatland_msgs::DebugTopicList& msg);

  /**
   * @brief Destruct
   */
  virtual ~FlatlandViz();

  rviz::VisualizationManager* manager_;

 private:
  rviz::RenderPanel* render_panel_;

  rviz::Display* grid_;
  rviz::Display* interactive_markers_;
  std::map<std::string, rviz::Display*> debug_displays_;
  ros::Subscriber debug_topic_subscriber_;
  rviz::PropertyTreeWidget* tree_widget_;
  FlatlandWindow* parent_;

  QMenu* file_menu_;
  QMenu* recent_configs_menu_;
  QMenu* view_menu_;
  QMenu* delete_view_menu_;
  QMenu* plugins_menu_;

  QToolBar* toolbar_;

  QActionGroup* toolbar_actions_;
  std::map<QAction*, rviz::Tool*> action_to_tool_map_;
  std::map<rviz::Tool*, QAction*> tool_to_action_map_;
  bool show_choose_new_master_option_;

  QAction* add_tool_action_;
  QMenu* remove_tool_menu_;

  /// Indicates if the toolbar should be visible outside of fullscreen mode.
  bool toolbar_visible_;

  // protected Q_SLOTS:
  void fullScreenChange(bool hidden);

  void setDisplayConfigModified();
  void addTool(rviz::Tool*);
  void removeTool(rviz::Tool*);
  void refreshTool(rviz::Tool*);
  void indicateToolIsCurrent(rviz::Tool*);
  void onToolbarActionTriggered(QAction* action);
  void onToolbarRemoveTool(QAction* remove_tool_menu_action);
  void initToolbars();
  void initMenus();
  void openNewToolDialog();
  void setFullScreen(bool full_screen);
};

#endif  // FLATLAND_VIZ_FLATLAND_VIZ_H