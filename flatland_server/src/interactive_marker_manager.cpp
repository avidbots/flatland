#include <flatland_server/interactive_marker_manager.h>

namespace flatland_server {

InteractiveMarkerManager::InteractiveMarkerManager(
    std::vector<Model *> *model_array_ptr) {
  models_ = model_array_ptr;
  interactive_marker_server_.reset(
      new interactive_markers::InteractiveMarkerServer(
          "interactive_model_markers"));
}

void InteractiveMarkerManager::createInteractiveMarker(
    const std::string &model_name, const Pose &pose,
    const visualization_msgs::MarkerArray &body_markers) {
  // Set up interactive marker control object to allow both translation and
  // rotation movement
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation.w = 1.0;
  control.orientation.y = 1.0;
  control.name = "move_xy_yaw";
  control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;

  // Add a cube marker a little forward from the origin of the interactive
  // marker. This guarantees an easy to manipulate target regardless of the body
  // geometry
  visualization_msgs::Marker easy_to_click_cube;
  easy_to_click_cube.type = visualization_msgs::Marker::CUBE;
  easy_to_click_cube.color.r = 0.0;
  easy_to_click_cube.color.g = 1.0;
  easy_to_click_cube.color.b = 0.0;
  easy_to_click_cube.color.a = 0.5;
  easy_to_click_cube.scale.x = 0.5;
  easy_to_click_cube.scale.y = 0.5;
  easy_to_click_cube.scale.z = 0.05;
  easy_to_click_cube.pose.position.x = 0.25;
  control.markers.push_back(easy_to_click_cube);

  // Add body markers to interactive marker control as well.
  for (size_t i = 0; i < body_markers.markers.size(); i++) {
    RotateTranslate rt = Geometry::CreateTransform(pose.x, pose.y, pose.theta);
    visualization_msgs::Marker transformed_body_marker =
        body_markers.markers[i];

    // Transform original body frame marker from global to local frame
    transformed_body_marker.header.frame_id = "";
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
    control.markers.push_back(transformed_body_marker);
  }

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

  // Send new interactive marker to server
  visualization_msgs::InteractiveMarker new_interactive_marker;
  new_interactive_marker.header.frame_id = "map";
  new_interactive_marker.name = model_name;
  new_interactive_marker.pose.position.x = pose.x;
  new_interactive_marker.pose.position.y = pose.y;
  new_interactive_marker.pose.orientation.w = cos(0.5 * pose.theta);
  new_interactive_marker.pose.orientation.z = sin(0.5 * pose.theta);
  new_interactive_marker.controls.push_back(control);
  new_interactive_marker.controls.push_back(no_control);

  interactive_marker_server_->insert(new_interactive_marker);
  interactive_marker_server_->setCallback(
      new_interactive_marker.name,
      boost::bind(&InteractiveMarkerManager::processInteractiveFeedback, this,
                  _1),
      visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::deleteInteractiveMarker(
    const std::string &model_name) {
  interactive_marker_server_->erase(model_name);
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::processInteractiveFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  // Update model that was manipulated
  for (int i = 0; i < models_->size(); i++) {
    if ((*models_)[i]->GetName() == feedback->marker_name) {
      // move the model
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
  interactive_marker_server_->applyChanges();
}

void InteractiveMarkerManager::update(const std::vector<Model *> &models_) {
  for (size_t i = 0; i < models_.size(); i++) {
    Pose pose;
    pose.x = models_[i]->bodies_[0]->physics_body_->GetPosition().x;
    pose.y = models_[i]->bodies_[0]->physics_body_->GetPosition().y;
    pose.theta = models_[i]->bodies_[0]->physics_body_->GetAngle();

    geometry_msgs::Pose new_pose;
    new_pose.position.x = pose.x;
    new_pose.position.y = pose.y;
    new_pose.orientation.w = cos(0.5 * pose.theta);
    new_pose.orientation.z = sin(0.5 * pose.theta);
    interactive_marker_server_->setPose(models_[i]->GetName(), new_pose);
    interactive_marker_server_->applyChanges();
  }
}
}
