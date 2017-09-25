/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   flatland_window.cpp
 * @brief  Main window and toolbars for flatland_viz
 * @author Joseph Duchesne
 * @author Mike Brousseau
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

#include "flatland_viz/flatland_window.h"

#include <QAction>
#include <QActionGroup>
#include <QDesktopWidget>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QStatusBar>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>

#include "rviz/display.h"
#include "rviz/display_context.h"
#include "rviz/displays_panel.h"
#include "rviz/env_config.h"
#include "rviz/failed_panel.h"
#include "rviz/help_panel.h"
#include "rviz/load_resource.h"
#include "rviz/loading_dialog.h"
#include "rviz/new_object_dialog.h"
#include "rviz/panel_dock_widget.h"
#include "rviz/panel_factory.h"
#include "rviz/properties/status_property.h"
#include "rviz/render_panel.h"
#include "rviz/screenshot_dialog.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/selection_panel.h"
#include "rviz/splash_screen.h"
#include "rviz/time_panel.h"
#include "rviz/tool.h"
#include "rviz/tool_manager.h"
#include "rviz/tool_properties_panel.h"
#include "rviz/view_manager.h"
#include "rviz/views_panel.h"
#include "rviz/visualization_frame.h"
#include "rviz/visualization_manager.h"
#include "rviz/widget_geometry_change_detector.h"
#include "rviz/yaml_config_reader.h"
#include "rviz/yaml_config_writer.h"

#include <OgreMeshManager.h>
#include <OgreRenderWindow.h>
//#include <ogre_helpers/initialization.h>

void FlatlandWindow::openNewToolDialog() {
  QString class_id;
  QStringList empty;
}

rviz::VisualizationManager *FlatlandWindow::getManager() {
  return visualization_manager_;
}

FlatlandWindow::FlatlandWindow(QWidget *parent) : QMainWindow(parent) {
  // Create the main viewport
  viz_ = new FlatlandViz(this);
  setCentralWidget(viz_);
  resize(QDesktopWidget().availableGeometry(this).size() * 0.9);

  // Configure the status bar
  fps_label_ = new QLabel("");
  fps_label_->setMinimumWidth(40);
  fps_label_->setAlignment(Qt::AlignRight);
  statusBar()->addPermanentWidget(fps_label_, 0);

  // Set the main window properties
  setWindowTitle("Flatland Viz");
  QFont font;
  font.setPointSize(font.pointSizeF() * 0.9);

  connect(viz_->manager_, &rviz::VisualizationManager::preUpdate, this,
          &FlatlandWindow::UpdateFps);
}

void FlatlandWindow::UpdateFps() {
  frame_count_++;
  ros::WallDuration wall_diff = ros::WallTime::now() - last_fps_calc_time_;

  if (wall_diff.toSec() > 1.0) {
    float fps = frame_count_ / wall_diff.toSec();
    frame_count_ = 0;
    last_fps_calc_time_ = ros::WallTime::now();

    fps_label_->setText(QString::number(int(fps)) + QString(" fps"));
  }
}
