#ifndef FLATLAND_VIZ_FLATLAND_VIZ_H
#define FLATLAND_VIZ_FLATLAND_VIZ_H

#include <ros/ros.h>
#include <QWidget>
#include <set>
#include "flatland_server/DebugTopicList.h"

namespace rviz {
class Display;
class RenderPanel;
class VisualizationManager;
}

class FlatlandWindow;

class FlatlandViz : public QWidget {
  Q_OBJECT
 public:
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
  void RecieveDebugTopics(const flatland_server::DebugTopicList::ConstPtr& msg);

  /**
   * @brief Destruct
   */
  virtual ~FlatlandViz();

  rviz::VisualizationManager* manager_;

 private:
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;
  std::set<std::string> debug_topics_;
  ros::Subscriber debug_topic_subscriber_;
};

#endif  // FLATLAND_VIZ_FLATLAND_VIZ_H