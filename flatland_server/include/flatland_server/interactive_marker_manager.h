#ifndef INTERACTIVEMARKERMANAGER_H
#define INTERACTIVEMARKERMANAGER_H

#include <flatland_server/geometry.h>
#include <flatland_server/model.h>
#include <flatland_server/plugin_manager.h>
#include <flatland_server/types.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/MarkerArray.h>

namespace flatland_server {

class InteractiveMarkerManager {
 public:
  /**
   * @brief Constructor for the interactive marker manager class
   * @param[in] model_list_ptr Pointer to the list of models in the World class
   * @param[in] plugin_manager_ptr Pointer to the plugin manager in the World
   * class
   */
  InteractiveMarkerManager(std::vector<Model*>* model_list_ptr,
                           PluginManager* plugin_manager_ptr);

  /**
   * @brief Destructor for the interactive marker manager class
   */
  ~InteractiveMarkerManager();

  /**
   * @brief Add a new interactive marker when spawning a model
   * @param[in] model_name Name of the model being spawned
   * @param[in] pose Initial pose of the spawning model
   * @param[in] body_markers Marker array corresponding to new model bodies
   */
  void createInteractiveMarker(
      const std::string& model_name, const Pose& pose,
      const visualization_msgs::MarkerArray& body_markers);

  /**
   * @brief Remove interactive marker corresponding to a given model when
   * deleting it from the simulation
   * @param[in] model_name Name of the model being deleted
   */
  void deleteInteractiveMarker(const std::string& model_name);

  /**
   * @brief Update the interactive marker poses after running
   * physics update to synchronize the markers with the models
   */
  void update();

  bool isManipulating() { return manipulating_model_; }

 private:
  interactive_markers::MenuHandler
      menu_handler_;  ///< Handler for the interactive marker context menus
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_marker_server_;  ///< Interactive marker server
  std::vector<Model*>*
      models_;  ///< Pointer to the model list in the World class
  PluginManager*
      plugin_manager_;  ///< Pointer to the plugin manager in the World class
  bool manipulating_model_;  ///< Boolean flag indicating if the user is
  /// manipulating a model with its interactive marker
  ros::WallTime pose_update_stamp_;  ///< Timestamp of the last received pose
  /// update feedback. Used to handle when the
  /// interactive marker server stops
  /// manipulating without triggering a
  /// MOUSE_UP event.

  /**
  * @brief Process interactive feedback on a MOUSE_UP event and use it
  * to move the appropriate model to the new pose
  * @param[in] feedback The feedback structure containing the name of the
  * manipulated model and the new pose
  */
  void processMouseUpFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
  * @brief Process interactive feedback on a MOUSE_DOWN event and use it
  * to set a flag indicating that the user is manipulating a model with
  * its interactive marker
  * @param[in] feedback The feedback structure containing the name of the
  * manipulated model and the current pose. Not used in this callback
  */
  void processMouseDownFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
  * @brief Process interactive feedback on a POSE_UPDATE event and record
  * the current wall time to detect a timeout in the update method
  * @param[in] feedback The feedback structure containing the name of the
  * manipulated model and the current pose. Not used in this method
  */
  void processPoseUpdateFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
  * @brief Process feedback from the context menu of the interactive marker to
  * delete the appropriate model
  * @param[in] feedback The feedback structure containing the name of the model
  * to be deleted
  */
  void deleteModelMenuCallback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};
}

#endif  // INTERACTIVEMARKERMANAGER_H
