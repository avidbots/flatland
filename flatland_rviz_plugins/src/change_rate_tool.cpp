#include "flatland_rviz_plugins/change_rate_tool.hpp"

#include <flatland_msgs/srv/change_rate.hpp>
#include <memory>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/tool_manager.hpp>

#include "change_rate_dialog.hpp"

namespace flatland_rviz_plugins {

ChangeRateTool::ChangeRateTool() { shortcut_key_ = 'R'; }

ChangeRateTool::~ChangeRateTool() {}

void ChangeRateTool::onInitialize() {
  node_ = rclcpp::Node::make_shared("change_rate_tool");
  change_rate_service_ =
      node_->create_client<flatland_msgs::srv::ChangeRate>("change_rate");

  setName("Change Simulation Rate");
  setIcon(rviz_common::loadPixmap(
      "package://flatland_rviz_plugins/icons/time.png"));
}

void ChangeRateTool::setRate(double rate) {
  auto request = std::make_shared<flatland_msgs::srv::ChangeRate::Request>();
  request->rate = rate;
  auto result = change_rate_service_->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, result);
}

void ChangeRateTool::activate() {
  auto *model_dialog = new ChangeRateDialog(nullptr, context_, this);
  model_dialog->setModal(true);
  model_dialog->show();
}

void ChangeRateTool::deactivate() {}

}  // namespace flatland_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(flatland_rviz_plugins::ChangeRateTool, rviz_common::Tool)
