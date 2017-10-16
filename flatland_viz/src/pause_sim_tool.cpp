#include <flatland_viz/pause_sim_tool.h>

namespace flatland_viz {

PauseSimTool::PauseSimTool() {}

// Disconnect the service client when the tool's destructor is called
PauseSimTool::~PauseSimTool() { pause_service_.shutdown(); }

// When the tool is initially loaded, connect to the pause toggle service
void PauseSimTool::onInitialize() {
  pause_service_ = nh_.serviceClient<std_srvs::Empty>("toggle_pause");
  setName("Pause/Resume");
}

// Every time the user presses the tool's Rviz toolbar button, call the pause
// toggle service
void PauseSimTool::activate() {
  std_srvs::Empty empty;
  pause_service_.call(empty);
}

void PauseSimTool::deactivate() {}

}  // end namespace flatland_viz

// Tell pluginlib about the tool class
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(flatland_viz::PauseSimTool, rviz::Tool)
