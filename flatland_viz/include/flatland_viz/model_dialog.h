/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   model_dialog.h
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

#ifndef MODEL_DIALOG_H
#define MODEL_DIALOG_H

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

#include <flatland_msgs/SpawnModel.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <regex>
//#include <thread>
#include <signal.h>

class QCheckBox;
class QLabel;
class QErrorMessage;

class DialogOptionsWidget;

namespace fs = boost::filesystem;
using namespace flatland_server;

class ModelDialog : public QWidget {
  Q_OBJECT

 public:
  static QColor saved_color_;

  ModelDialog(QWidget* parent = 0);

 private Q_SLOTS:
  /**
   * @name        SetColor
   * @brief       Callback to pop up a ColorDialog
   */
  void SetColor();
  /**
   * @name        CancelButtonClicked
   * @brief       Callback to dismiss the model dialog (cancel was clicked)
   */
  void CancelButtonClicked();
  /**
   * @name        OkButtonClicked
   * @brief       Callback to create the model (ok was clicked)
   */
  void OkButtonClicked();
  /**
   * @name        SelectFile
   * @brief       Callback to choose model
   */
  QString SelectFile();
  /**
   * @name        SetButtonColor
   * @brief       Changes a button's color
   */
  void SetButtonColor(const QColor* c, QPushButton* b);
  /**
   * @name        SpawnModelClient
   * @brief       Makes a call to spawn model ros service
   */

  void SpawnModelClient();
  void SigintHandler(int sig);

 private:
  QPushButton* color_button;
  QString path_to_model_file;
  QLineEdit *x_edit, *y_edit, *a_edit, *n_edit;

 protected:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  boost::filesystem::path robot_yaml;
  // Timekeeper timekeeper;
  ros::NodeHandle nh;
  ros::ServiceClient client;
  flatland_msgs::SpawnModel srv;
  // std::thread simulation_thread;
  // bool stop_thread;
  World* w;
};

#endif