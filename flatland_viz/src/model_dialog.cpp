/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   model_dialog.cpp
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

#include <QColorDialog>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QVBoxLayout>

#include "flatland_viz/model_dialog.h"

// Initialize static variables
QColor ModelDialog::saved_color_;

QString ModelDialog::SelectFile() {
  QString fileName =
      QFileDialog::getOpenFileName(this, tr("Open model file"), "", "");
  if (fileName.isEmpty())
    return fileName;
  else {
    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly)) {
      QMessageBox::information(this, tr("Unable to open file"),
                               file.errorString());
      return fileName;
    }
    file.close();

    return fileName;
  }
}

ModelDialog::ModelDialog(QWidget *parent) : QDialog(parent) {
  ROS_ERROR_STREAM("ModelDialog::ModelDialog");

  path_to_model_file = SelectFile();
  QVBoxLayout *v_layout = new QVBoxLayout;

  // we are injecting horizontal layouts into the master vertical layout
  QHBoxLayout *h1_layout = new QHBoxLayout;
  QHBoxLayout *h2_layout = new QHBoxLayout;
  QHBoxLayout *h3_layout = new QHBoxLayout;
  QHBoxLayout *h4_layout = new QHBoxLayout;
  QHBoxLayout *h5_layout = new QHBoxLayout;

  QLabel *colorLabel = new QLabel;
  int frameStyle = QFrame::Sunken | QFrame::Panel;
  colorLabel->setFrameStyle(frameStyle);
  color_button = new QPushButton("");
  QPushButton *okButton = new QPushButton("ok");
  QPushButton *cancelButton = new QPushButton("cancel");
  connect(color_button, &QAbstractButton::clicked, this,
          &ModelDialog::SetColor);

  // path label and path
  QLabel *path = new QLabel;
  path->setText(path_to_model_file);
  h1_layout->addWidget(new QLabel("path:"));
  h1_layout->addWidget(path);

  // name label and name LineEdit
  n_edit = new QLineEdit;
  h2_layout->addWidget(new QLabel("name:"));
  h2_layout->addWidget(n_edit);

  // set the default name to the filename parsed using boost
  std::string bsfn =
      boost::filesystem::basename(path_to_model_file.toStdString());
  QString fn = QString::fromStdString(bsfn);
  n_edit->setText(fn);

  // color label and button
  h3_layout->addWidget(new QLabel("color"));
  h3_layout->addWidget(color_button);

  // first time use color
  // after that, use saved_color_
  static bool first_time = true;
  QColor color;
  if (first_time) {
    ROS_ERROR_STREAM("firstTime");
    first_time = false;
    color = Qt::blue;
  } else {
    ROS_ERROR_STREAM("anotherTime");
    color = saved_color_;
  }

  // x label and x LineEdit
  x_edit = new QLineEdit;
  h4_layout->addWidget(new QLabel("x:"));
  h4_layout->addWidget(x_edit);

  // y label and y LineEdit
  y_edit = new QLineEdit;
  h4_layout->addWidget(new QLabel("y:"));
  h4_layout->addWidget(y_edit);

  // a label and a LineEdit
  a_edit = new QLineEdit;
  h4_layout->addWidget(new QLabel("a:"));
  h4_layout->addWidget(a_edit);

  // ok button
  h5_layout->addWidget(okButton);
  connect(okButton, &QAbstractButton::clicked, this,
          &ModelDialog::OkButtonClicked);

  ModelDialog::SetButtonColor(&color, color_button);

  // cancel button
  h5_layout->addWidget(cancelButton);
  connect(cancelButton, &QAbstractButton::clicked, this,
          &ModelDialog::CancelButtonClicked);

  // add the horizontal layouts to the vertical layout
  v_layout->addLayout(h1_layout);
  v_layout->addLayout(h2_layout);
  v_layout->addLayout(h3_layout);
  v_layout->addLayout(h4_layout);
  v_layout->addLayout(h5_layout);

  // set the top level layout
  setLayout(v_layout);

  // delete the Dialog if the user clicks on the x to close window
  this->setAttribute(Qt::WA_DeleteOnClose, true);
}

void ModelDialog::CancelButtonClicked() {
  ROS_ERROR_STREAM("Cancel clicked");
  this->close();
}

void ModelDialog::OkButtonClicked() {
  ROS_ERROR_STREAM("Ok clicked");
  ROS_ERROR_STREAM("connect to ROS model service");

  ModelDialog::SpawnModelClient();
}

void ModelDialog::SetColor() {
  const QColorDialog::ColorDialogOptions options =
      (QColorDialog::ShowAlphaChannel);

  const QColor color =
      QColorDialog::getColor(Qt::green, this, "Select Color", options);
  if (color.isValid()) {
    color_button->setText(color.name());
  }

  ModelDialog::SetButtonColor(&color, color_button);
  saved_color_ = color;
}

void ModelDialog::SetButtonColor(const QColor *c, QPushButton *b) {
  b->setText(c->name());
  b->setPalette(QPalette(*c));
  b->setAutoFillBackground(true);
  QPalette pal = b->palette();
  pal.setColor(QPalette::Button, *c);
  QString qs = "background-color:" + c->name();
  color_button->setStyleSheet(qs);
}

void ModelDialog::SpawnModelClient() {
  srv.request.name = "service_manager_test_robot";
  srv.request.ns = n_edit->text().toStdString();
  srv.request.yaml_path = path_to_model_file.toStdString();
  srv.request.pose.x = x_edit->text().toFloat();
  srv.request.pose.y = y_edit->text().toFloat();
  srv.request.pose.theta = a_edit->text().toFloat();

  client = nh.serviceClient<flatland_msgs::SpawnModel>("spawn_model");

  client.call(srv);
}
