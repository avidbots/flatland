#include <flatland_server/interactive_marker_manager.h>

namespace flatland_server {

InteractiveMarkerManager::InteractiveMarkerManager(
    std::vector<Model *> *model_list_ptr, PluginManager *plugin_manager_ptr) {
  models_ = model_list_ptr;
  plugin_manager_ = plugin_manager_ptr;
  manipulating_model_ = false;

  // Initialize interactive marker server
  interactive_marker_server_.reset(
      new interactive_markers::InteractiveMarkerServer(
          "interactive_model_markers"));

  // Add "Delete Model" context menu option to menu handler and bind callback
  menu_handler_.setCheckState(
      menu_handler_.insert(
          "Delete Model",
          boost::bind(&InteractiveMarkerManager::deleteModelMenuCallback, this,
                      _1)),
      interactive_markers::MenuHandler::NO_CHECKBOX);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::createInteractiveMarker(
    const std::string &model_name, const Pose &pose,
    const visualization_msgs::MarkerArray &body_markers) {
  // Set up interactive marker control objects to allow both translation and
  // rotation movement
  visualization_msgs::InteractiveMarkerControl plane_control;
  plane_control.always_visible = true;
  plane_control.orientation.w = 0.707;
  plane_control.orientation.y = 0.707;
  plane_control.name = "move_xy";
  plane_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  visualization_msgs::InteractiveMarkerControl rotate_control;
  rotate_control.always_visible = true;
  rotate_control.orientation.w = 0.707;
  rotate_control.orientation.y = 0.707;
  rotate_control.name = "rotate_z";
  rotate_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

  // Add a non-interactive text marker with the model name
  visualization_msgs::InteractiveMarkerControl no_control;
  no_control.always_visible = true;
  no_control.name = "no_control";
  no_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::NONE;
  visualization_msgs::Marker text_marker;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.pose.position.x = 0.5;
  text_marker.pose.position.y = 0.0;
  text_marker.scale.z = 0.5;
  text_marker.text = model_name;
  no_control.markers.push_back(text_marker);

  // Add a cube marker to be an easy-to-manipulate target in Rviz
  visualization_msgs::Marker easy_to_click_cube;
  easy_to_click_cube.type = visualization_msgs::Marker::CUBE;
  easy_to_click_cube.color.r = 0.0;
  easy_to_click_cube.color.g = 1.0;
  easy_to_click_cube.color.b = 0.0;
  easy_to_click_cube.color.a = 0.5;
  easy_to_click_cube.scale.x = 0.5;
  easy_to_click_cube.scale.y = 0.5;
  easy_to_click_cube.scale.z = 0.05;
  easy_to_click_cube.pose.position.x = 0.0;
  plane_control.markers.push_back(easy_to_click_cube);

  // Also add body markers to the no_control object to visualize model pose
  // while moving its interactive marker
  for (size_t i = 0; i < body_markers.markers.size(); i++) {
    visualization_msgs::Marker transformed_body_marker =
        body_markers.markers[i];

    // Transform original body frame marker from global to local frame
    RotateTranslate rt = Geometry::CreateTransform(pose.x, pose.y, pose.theta);
    transformed_body_marker.header.frame_id = "";
    transformed_body_marker.header.stamp = ros::Time(0);
    transformed_body_marker.pose.position.x =
        (body_markers.markers[i].pose.position.x - rt.dx) * rt.cos +
        (body_markers.markers[i].pose.position.y - rt.dy) * rt.sin;
    transformed_body_marker.pose.position.y =
        -(body_markers.markers[i].pose.position.x - rt.dx) * rt.sin +
        (body_markers.markers[i].pose.position.y - rt.dy) * rt.cos;
    transformed_body_marker.pose.orientation.w = 1.0;
    transformed_body_marker.pose.orientation.x = 0.0;
    transformed_body_marker.pose.orientation.y = 0.0;
    transformed_body_marker.pose.orientation.z = 0.0;

    // Make line strips thicker than the original
    if (transformed_body_marker.type ==
            visualization_msgs::Marker::LINE_STRIP ||
        transformed_body_marker.type == visualization_msgs::Marker::LINE_LIST) {
      transformed_body_marker.scale.x = 0.1;
    }

    // Add transformed body marker to interactive marker control object
    no_control.markers.push_back(transformed_body_marker);
  }

  // Send new interactive marker to server
  visualization_msgs::InteractiveMarker new_interactive_marker;
  new_interactive_marker.header.frame_id = "map";
  new_interactive_marker.header.stamp = ros::Time(0);
  new_interactive_marker.name = model_name;
  new_interactive_marker.pose.position.x = pose.x;
  new_interactive_marker.pose.position.y = pose.y;
  new_interactive_marker.pose.orientation.w = cos(0.5 * pose.theta);
  new_interactive_marker.pose.orientation.z = sin(0.5 * pose.theta);
  new_interactive_marker.controls.push_back(plane_control);
  new_interactive_marker.controls.push_back(rotate_control);
  new_interactive_marker.controls.push_back(no_control);
  interactive_marker_server_->insert(new_interactive_marker);

  // Bind feedback callbacks for the new interactive marker
  interactive_marker_server_->setCallback(
      model_name,
      boost::bind(&InteractiveMarkerManager::processMouseUpFeedback, this, _1),
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
  interactive_marker_server_->setCallback(
      model_name,
      boost::bind(&InteractiveMarkerManager::processMouseDownFeedback, this,
                  _1),
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN);
  interactive_marker_server_->setCallback(
      model_name,
      boost::bind(&InteractiveMarkerManager::processPoseUpdateFeedback, this,
                  _1),
      visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);

  // Add context menu to the new interactive marker
  menu_handler_.apply(*interactive_marker_server_, model_name);

  // Apply changes to server
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::deleteModelMenuCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // Delete the model just as when the DeleteModel service is called
  for (unsigned int i = 0; i < (*models_).size(); i++) {
    if ((*models_)[i]->GetName() == feedback->marker_name) {
      // delete the plugins associated with the model
      plugin_manager_->DeleteModelPlugin((*models_)[i]);
      delete (*models_)[i];
      (*models_).erase((*models_).begin() + i);

      // Also remove corresponding interactive marker
      deleteInteractiveMarker(feedback->marker_name);
      break;
    }
  }

  // Update menu handler and server
  menu_handler_.apply(*interactive_marker_server_, feedback->marker_name);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::deleteInteractiveMarker(
    const std::string &model_name) {
  // Remove target interactive marker by name and
  // update the server
  interactive_marker_server_->erase(model_name);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::processMouseUpFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // Update model that was manipulated the same way
  // as when the MoveModel service is called
  for (unsigned int i = 0; i < models_->size(); i++) {
    if ((*models_)[i]->GetName() == feedback->marker_name) {
      Pose new_pose;
      new_pose.x = feedback->pose.position.x;
      new_pose.y = feedback->pose.position.y;
      new_pose.theta = atan2(
          2.0 * feedback->pose.orientation.w * feedback->pose.orientation.z,
          1.0 -
              2.0 * feedback->pose.orientation.z *
                  feedback->pose.orientation.z);
      (*models_)[i]->SetPose(new_pose);
      break;
    }
  }
  manipulating_model_ = false;
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::processMouseDownFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  manipulating_model_ = true;
}

void InteractiveMarkerManager::processPoseUpdateFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  pose_update_stamp_ = ros::WallTime::now();
}

void InteractiveMarkerManager::update() {
  // Loop through each model, extract the pose of the root body,
  // and use it to update the interactive marker pose. Only
  // necessary to compute if user is not currently dragging
  // an interactive marker
  if (!manipulating_model_) {
    for (size_t i = 0; i < (*models_).size(); i++) {
      geometry_msgs::Pose new_pose;
      new_pose.position.x =
          (*models_)[i]->bodies_[0]->physics_body_->GetPosition().x;
      new_pose.position.y =
          (*models_)[i]->bodies_[0]->physics_body_->GetPosition().y;
      double theta = (*models_)[i]->bodies_[0]->physics_body_->GetAngle();
      new_pose.orientation.w = cos(0.5 * theta);
      new_pose.orientation.z = sin(0.5 * theta);
      interactive_marker_server_->setPose((*models_)[i]->GetName(), new_pose);
      interactive_marker_server_->applyChanges();
    }
  }

  // Detect when interaction stops without triggering a MOUSE_UP event by
  // monitoring the time since the last pose update feedback, which comes
  // in at 33 Hz if the user is dragging the marker.  When the stream of
  // pose update feedback stops, automatically clear the manipulating_model_
  // flag to unpause the simulation.
  double dt = 0;
  try {
    dt = (ros::WallTime::now() - pose_update_stamp_).toSec();
  } catch (std::runtime_error &ex) {
    ROS_ERROR(
        "Flatland Interactive Marker Manager runtime error: (%f - %f) [%s]",
        ros::WallTime::now().toSec(), pose_update_stamp_.toSec(), ex.what());
  }
  if (manipulating_model_ && dt > 0.1 && dt < 1.0) {
    manipulating_model_ = false;
  }
}

InteractiveMarkerManager::~InteractiveMarkerManager() {
  interactive_marker_server_.reset();
}
}
