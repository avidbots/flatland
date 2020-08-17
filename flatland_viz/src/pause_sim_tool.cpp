#include <flatland_viz/pause_sim_tool.h>

namespace flatland_viz {

PauseSimTool::PauseSimTool() {}

// Disconnect the service client when the tool's destructor is called
PauseSimTool::~PauseSimTool() { rclcpp::shutdown(); /*pause_service_->shutdown();*/ }

// When the tool is initially loaded, connect to the pause toggle service
void PauseSimTool::onInitialize() {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pause_sim_tool");  // TODO
    pause_service_ = node->create_client<std_srvs::srv::Empty>("toggle_pause");
    setName("Pause/Resume");
}

// Every time the user presses the tool's Rviz toolbar button, call the pause
// toggle service
void PauseSimTool::activate() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto result = pause_service_->async_send_request(request);
}

void PauseSimTool::deactivate() {}

}  // end namespace flatland_viz

// Tell pluginlib about the tool class
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(flatland_viz::PauseSimTool, rviz_common::Tool)
