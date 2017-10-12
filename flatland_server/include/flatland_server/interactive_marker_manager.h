#ifndef INTERACTIVEMARKERMANAGER_H
#define INTERACTIVEMARKERMANAGER_H

#include <flatland_server/geometry.h>
#include <flatland_server/model.h>
#include <flatland_server/types.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/MarkerArray.h>

namespace flatland_server {

class InteractiveMarkerManager {
 public:
  InteractiveMarkerManager(std::vector<Model*>* model_array_ptr);

  void createInteractiveMarker(
      const std::string& model_name, const Pose& pose,
      const visualization_msgs::MarkerArray& body_markers);

  void deleteInteractiveMarker(const std::string& model_name);

  void update(const std::vector<Model*>& models);

 private:
  void processInteractiveFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_marker_server_;

  std::vector<Model*>* models_;
};
}

#endif  // INTERACTIVEMARKERMANAGER_H
