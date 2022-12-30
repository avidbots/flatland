/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   load_model_dialog.h
 * @brief  Spawn dialog
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

#ifndef FLATLAND_RVIZ_PLUGINS__LOAD_MODEL_DIALOG_HPP_
#define FLATLAND_RVIZ_PLUGINS__LOAD_MODEL_DIALOG_HPP_

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>

#include <QCursor>
#include <QDialog>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>
#include <boost/filesystem.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/geometry.hpp>
#include <rviz_rendering/mesh_loader.hpp>

#include "flatland_rviz_plugins/spawn_model_tool.hpp"

namespace flatland_rviz_plugins
{

class LoadModelDialog : public QDialog
{
public:
  /**
   * @name                LoadModelDialog
   * @brief               Launch load model dialog
   * @param               parent, parent widget pointer
   * @param               tool, pointer to this so dialog can call methods
   */
  LoadModelDialog(QWidget * parent, rviz_common::DisplayContext * context, SpawnModelTool * tool);

private:
  rviz_common::DisplayContext * context_;

  /**
   * @name                ChooseFile
   * @brief               Launch file selection dialog
   */
  QString ChooseFile();
  /**
   * @name                AddNumberAndUpdateName
   * @brief               Add numbering to name and show in the name widget
   */
  void AddNumberAndUpdateName();

  static QString path_to_model_file;  // full path to model file
  static int count;                   // counter for adding unique number to filename
  static bool numbering;              // flag to use unique numbering

  SpawnModelTool * tool_;
  QLineEdit * n_edit;      // name lineEdit widget
  QLabel * p_label;        // path label widget
  QCheckBox * n_checkbox;  // checkbox widget

public Q_SLOTS:
  /**
   * @name                NumberCheckBoxChanged
   * @brief               Checkbox was clicked, toggle numbering of names
   */
  void NumberCheckBoxChanged(bool value);
  /**
   * @name                CancelButtonClicked
   * @brief               Cancel button was clicked, dismiss dialog
   */
  void CancelButtonClicked();
  /**
   * @name                OkButtonClicked
   * @brief               Ok button was clicked, begin placement
   */
  void OkButtonClicked();
  /**
   * @name                PathButtonClicked
   * @brief               Path button was clicked, launch file selection dialog
   */
  void on_PathButtonClicked();
};

}  // namespace flatland_rviz_plugins

#endif  // FLATLAND_RVIZ_PLUGINS__LOAD_MODEL_DIALOG_HPP_
