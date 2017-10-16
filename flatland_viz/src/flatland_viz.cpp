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

#include <QAction>
#include <QApplication>
#include <QCloseEvent>
#include <QDesktopServices>
#include <QDir>
#include <QDockWidget>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QShortcut>
#include <QStatusBar>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QToolButton>
#include <QUrl>

#include <ros/ros.h>
#include <stdlib.h>

#include "rviz/display.h"
#include "rviz/render_panel.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"

#include "flatland_viz/flatland_window.h"

#include "flatland_viz/flatland_viz.h"

// Constructor.
FlatlandViz::FlatlandViz(FlatlandWindow* parent) : QWidget((QWidget*)parent) {
  parent_ = parent;
  toolbar_ = parent->addToolBar("Tools");

  // init toolbar action handler
  initToolbars();

  initMenus();

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
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

  // bind toolbar events
  rviz::ToolManager* tool_man = manager_->getToolManager();

  connect(manager_, SIGNAL(configChanged()), this,
          SLOT(setDisplayConfigModified()));
  connect(tool_man, &rviz::ToolManager::toolAdded, this, &FlatlandViz::addTool);
  connect(tool_man, SIGNAL(toolRemoved(rviz::Tool*)), this,
          SLOT(removeTool(rviz::Tool*)));
  connect(tool_man, SIGNAL(toolRefreshed(rviz::Tool*)), this,
          SLOT(refreshTool(rviz::Tool*)));
  connect(tool_man, SIGNAL(toolChanged(rviz::Tool*)), this,
          SLOT(indicateToolIsCurrent(rviz::Tool*)));

  manager_->initialize();

  tool_man->addTool("flatland_viz/SpawnModel");
  tool_man->addTool("flatland_viz/PauseSim");

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

  // Create interactive markers display
  interactive_markers_ =
      manager_->createDisplay("rviz/InteractiveMarkers", "Move Objects", false);
  if (interactive_markers_ == nullptr) {
    ROS_FATAL("Interactive markers failed to instantiate");
    exit(1);
  }
  interactive_markers_->subProp("Update Topic")
      ->setValue("/interactive_model_markers/update");

  // Subscribe to debug topics topic
  ros::NodeHandle n;
  debug_topic_subscriber_ = n.subscribe("/flatland_server/debug/topics", 0,
                                        &FlatlandViz::RecieveDebugTopics, this);
}

// Destructor.
FlatlandViz::~FlatlandViz() {
  delete render_panel_;
  delete manager_;
}

void FlatlandViz::indicateToolIsCurrent(rviz::Tool* tool) {
  QAction* action = tool_to_action_map_[tool];
  if (action) {
    action->setChecked(true);
  }
}

void FlatlandViz::setDisplayConfigModified() {
  ROS_ERROR("setDisplayConfigModified called");
}

void FlatlandViz::addTool(rviz::Tool* tool) {
  ROS_ERROR("addTool called");
  QAction* action = new QAction(tool->getName(), toolbar_actions_);
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
  action->setCheckable(true);
  toolbar_->addAction(action);
  action_to_tool_map_[action] = tool;
  tool_to_action_map_[tool] = action;

  remove_tool_menu_->addAction(tool->getName());
}

void FlatlandViz::onToolbarActionTriggered(QAction* action) {
  ROS_ERROR("onToolbarActionTriggered called");

  rviz::Tool* current_tool = manager_->getToolManager()->getCurrentTool();
  rviz::Tool* tool = action_to_tool_map_[action];

  if (tool) {
    manager_->getToolManager()->setCurrentTool(tool);

    // If the simulation pause/resume tool was clicked, automatically and
    // immediately switch back to the previously active tool
    if (tool->getClassId().toStdString() == "flatland_viz/PauseSim") {
      manager_->getToolManager()->setCurrentTool(current_tool);
      tool = current_tool;
      indicateToolIsCurrent(tool);
    }

    // Show or hide interactive markers depending on whether interact mode is
    // active
    if (tool->getClassId().toStdString() == "rviz/Interact") {
      interactive_markers_->setEnabled(true);
    } else {
      interactive_markers_->setEnabled(false);
    }
  }
}

void FlatlandViz::removeTool(rviz::Tool* tool) {
  ROS_ERROR("removeTool called");
  QAction* action = tool_to_action_map_[tool];
  if (action) {
    toolbar_actions_->removeAction(action);
    toolbar_->removeAction(action);
    tool_to_action_map_.erase(tool);
    action_to_tool_map_.erase(action);
  }
  QString tool_name = tool->getName();
  QList<QAction*> remove_tool_actions = remove_tool_menu_->actions();
  for (int i = 0; i < remove_tool_actions.size(); i++) {
    ROS_ERROR_STREAM("Removing --------> " << tool_name.toStdString());
    QAction* removal_action = remove_tool_actions.at(i);
    if (removal_action->text() == tool_name) {
      remove_tool_menu_->removeAction(removal_action);
      break;
    }
  }
}

void FlatlandViz::initMenus() {
  file_menu_ = parent_->menuBar()->addMenu("&File");

  QAction* file_menu_open_action = file_menu_->addAction(
      "&Open Config", this, SLOT(onOpen()), QKeySequence("Ctrl+O"));
  this->addAction(file_menu_open_action);
  QAction* file_menu_save_action = file_menu_->addAction(
      "&Save Config", this, SLOT(onSave()), QKeySequence("Ctrl+S"));
  this->addAction(file_menu_save_action);
  QAction* file_menu_save_as_action = file_menu_->addAction(
      "Save Config &As", this, SLOT(onSaveAs()), QKeySequence("Ctrl+Shift+S"));
  this->addAction(file_menu_save_as_action);

  recent_configs_menu_ = file_menu_->addMenu("&Recent Configs");
  file_menu_->addAction("Save &Image", this, SLOT(onSaveImage()));
  if (show_choose_new_master_option_) {
    file_menu_->addSeparator();
    file_menu_->addAction("Change &Master", this, SLOT(changeMaster()));
  }
  file_menu_->addSeparator();

  QAction* file_menu_quit_action = file_menu_->addAction(
      "&Quit", this, SLOT(close()), QKeySequence("Ctrl+Q"));
  this->addAction(file_menu_quit_action);

  view_menu_ = parent_->menuBar()->addMenu("&Panels");
  view_menu_->addAction("Add &New Panel", this, SLOT(openNewPanelDialog()));
  delete_view_menu_ = view_menu_->addMenu("&Delete Panel");
  delete_view_menu_->setEnabled(false);

  QAction* fullscreen_action = view_menu_->addAction(
      "&Fullscreen", this, SLOT(setFullScreen(bool)), Qt::Key_F11);
  fullscreen_action->setCheckable(true);
  this->addAction(fullscreen_action);  // Also add to window, or the shortcut
                                       // doest work when the menu is hidden.

  // connect(this, SIGNAL(fullScreenChange(bool)), fullscreen_action,
  //        SLOT(setChecked(bool)));
  new QShortcut(Qt::Key_Escape, this, SLOT(exitFullScreen()));
  view_menu_->addSeparator();

  QMenu* help_menu = parent_->menuBar()->addMenu("&Help");
  help_menu->addAction("Show &Help panel", this, SLOT(showHelpPanel()));
  help_menu->addAction("Open rviz wiki in browser", this, SLOT(onHelpWiki()));
  help_menu->addSeparator();
  help_menu->addAction("&About", this, SLOT(onHelpAbout()));
}

void FlatlandViz::initToolbars() {
  QFont font;
  font.setPointSize(font.pointSizeF() * 0.9);

  // make toolbar with plugin tools

  toolbar_->setFont(font);
  toolbar_->setContentsMargins(0, 0, 0, 0);
  toolbar_->setObjectName("Tools");
  toolbar_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  toolbar_actions_ = new QActionGroup(this);
  connect(toolbar_actions_, &QActionGroup::triggered, this,
          &FlatlandViz::onToolbarActionTriggered);

  add_tool_action_ = new QAction("", toolbar_actions_);
  add_tool_action_->setToolTip("Add a new tool");
  add_tool_action_->setIcon(rviz::loadPixmap("package://rviz/icons/plus.png"));
  toolbar_->addAction(add_tool_action_);

  connect(add_tool_action_, &QAction::triggered, this,
          &FlatlandViz::openNewToolDialog);

  remove_tool_menu_ = new QMenu();
  QToolButton* remove_tool_button = new QToolButton();
  remove_tool_button->setMenu(remove_tool_menu_);
  remove_tool_button->setPopupMode(QToolButton::InstantPopup);
  remove_tool_button->setToolTip("Remove a tool from the toolbar");
  remove_tool_button->setIcon(
      rviz::loadPixmap("package://rviz/icons/minus.png"));
  toolbar_->addWidget(remove_tool_button);

  connect(remove_tool_menu_, &QMenu::triggered, this,
          &FlatlandViz::onToolbarRemoveTool);
}

void FlatlandViz::openNewToolDialog() {
  ROS_ERROR("openNewToolDialog called");
  QString class_id;
  QStringList empty;
  rviz::ToolManager* tool_man = manager_->getToolManager();

  rviz::NewObjectDialog* dialog =
      new rviz::NewObjectDialog(tool_man->getFactory(), "Tool", empty,
                                tool_man->getToolClasses(), &class_id);
  manager_->stopUpdate();
  if (dialog->exec() == QDialog::Accepted) {
    tool_man->addTool(class_id);
  }
  manager_->startUpdate();
  activateWindow();  // Force keyboard focus back on main window.
}

void FlatlandViz::onToolbarRemoveTool(QAction* remove_tool_menu_action) {
  ROS_ERROR("onToolbarRemoveTool called");
  QString name = remove_tool_menu_action->text();
  for (int i = 0; i < manager_->getToolManager()->numTools(); i++) {
    rviz::Tool* tool = manager_->getToolManager()->getTool(i);
    if (tool->getName() == name) {
      ROS_ERROR_STREAM("Removing --------> " << name.toStdString());
      manager_->getToolManager()->removeTool(i);
      removeTool(tool);
      return;
    }
  }
}

void FlatlandViz::refreshTool(rviz::Tool* tool) {
  QAction* action = tool_to_action_map_[tool];
  action->setIcon(tool->getIcon());
  action->setIconText(tool->getName());
}

void FlatlandViz::setFullScreen(bool full_screen) {
  // Q_EMIT(fullScreenChange(full_screen));

  if (full_screen) toolbar_visible_ = toolbar_->isVisible();
  parent_->menuBar()->setVisible(!full_screen);
  toolbar_->setVisible(!full_screen && toolbar_visible_);
  parent_->statusBar()->setVisible(!full_screen);

  if (full_screen)
    setWindowState(windowState() | Qt::WindowFullScreen);
  else
    setWindowState(windowState() & ~Qt::WindowFullScreen);
  show();
}

void FlatlandViz::RecieveDebugTopics(const flatland_msgs::DebugTopicList& msg) {
  std::vector<std::string> topics = msg.topics;

  // check for deleted topics
  for (auto& topic : debug_displays_) {
    if (std::count(topics.begin(), topics.end(), topic.first) == 0) {
      delete debug_displays_[topic.first];
      debug_displays_.erase(topic.first);
    }
  }

  // check for new topics
  for (const auto& topic : topics) {
    if (debug_displays_.count(topic) == 0) {
      // Create the marker display and set its topic
      debug_displays_[topic] = manager_->createDisplay(
          "rviz/MarkerArray", QString::fromLocal8Bit(topic.c_str()), true);
      if (debug_displays_[topic] == nullptr) {
        ROS_FATAL("MarkerArray failed to instantiate");
        exit(1);
      }
      QString topic_qt = QString::fromLocal8Bit(
          (std::string("/flatland_server/debug/") + topic).c_str());
      debug_displays_[topic]->subProp("Marker Topic")->setValue(topic_qt);
    }
  }
}
