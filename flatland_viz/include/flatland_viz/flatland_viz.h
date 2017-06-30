#ifndef FLATLAND_VIZ_FLATLAND_VIZ_H
#define FLATLAND_VIZ_FLATLAND_VIZ_H

#include <QWidget>

namespace rviz {
class Display;
class RenderPanel;
class VisualizationManager;
}

class FlatlandViz : public QWidget {
  Q_OBJECT
 public:
  FlatlandViz(QWidget* parent = 0);
  virtual ~FlatlandViz();

 private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;
};

#endif  // FLATLAND_VIZ_FLATLAND_VIZ_H