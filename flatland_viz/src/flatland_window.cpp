#include "flatland_viz/flatland_window.h"
#include <QDesktopWidget>
#include <QWidget>

FlatlandWindow::FlatlandWindow(QWidget *parent) : QMainWindow(parent) {
  // Create the main viewport
  viz_ = new FlatlandViz(this);
  setCentralWidget(viz_);
  resize(QDesktopWidget().availableGeometry(this).size() * 0.9);

  // Todo: configure the toolbar

  setWindowTitle("Flatland Viz");
}