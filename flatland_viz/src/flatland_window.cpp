#include "flatland_viz/flatland_window.h"
#include <QDesktopWidget>
#include <QLabel>
#include <QStatusBar>
#include <QToolButton>
#include <QWidget>
#include "rviz/visualization_manager.h"

FlatlandWindow::FlatlandWindow(QWidget* parent) : QMainWindow(parent) {
  // Create the main viewport
  viz_ = new FlatlandViz(this);
  setCentralWidget(viz_);
  resize(QDesktopWidget().availableGeometry(this).size() * 0.9);

  // Todo: configure the toolbar

  // Configure the status bar
  fps_label_ = new QLabel("");
  fps_label_->setMinimumWidth(40);
  fps_label_->setAlignment(Qt::AlignRight);
  statusBar()->addPermanentWidget(fps_label_, 0);

  // Set the title
  setWindowTitle("Flatland Viz");

  // Register for updates
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