#include "flatland_rviz_plugins/pause_tool.hpp"

#include <memory>
#include <rviz_common/display_context.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/tool_manager.hpp>

namespace flatland_rviz_plugins {

PauseTool::PauseTool() { shortcut_key_ = ' '; }

PauseTool::~PauseTool() {}

void PauseTool::onInitialize() {
  node_ = rclcpp::Node::make_shared("pause_sim_tool");
  pause_service_ = node_->create_client<std_srvs::srv::Empty>("toggle_pause");

  setName("Toggle Pause");
  setIcon(rviz_common::loadPixmap(
      "package://flatland_rviz_plugins/icons/pause.svg"));
}

void PauseTool::activate() {
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = pause_service_->async_send_request(request);
  rclcpp::spin_until_future_complete(node_, result);

  auto tool_man = context_->getToolManager();
  auto dflt_tool = tool_man->getDefaultTool();
  tool_man->setCurrentTool(dflt_tool);
}

void PauseTool::deactivate() {}

}  // namespace flatland_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(flatland_rviz_plugins::PauseTool, rviz_common::Tool)
