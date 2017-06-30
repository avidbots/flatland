#include <QMainWindow>
#include <QWidget>
#include "flatland_viz/flatland_viz.h"

class FlatlandWindow : public QMainWindow {
  Q_OBJECT
 public:
  FlatlandWindow(QWidget* parent = 0);

 private:
  FlatlandViz* viz_;
};