#include "include/flatland_rviz_plugins/interactive_tool.hpp"

#include <rviz_common/load_resource.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/display_group.hpp>

namespace flatland_rviz_plugins {

InteractiveTool::InteractiveTool() : rviz_default_plugins::tools::InteractionTool() {
}

void InteractiveTool::enableMarkers(bool is_enabled) {
  auto root_display = context_->getRootDisplayGroup();
  for (int i = 0; i < root_display->numDisplays(); ++i) {
    auto display = root_display->getDisplayAt(i);
    if (display->getClassId() == "rviz_default_plugins/InteractiveMarkers") {
      display->setEnabled(is_enabled);
    }
  }
}

void InteractiveTool::onInitialize() {
  rviz_default_plugins::tools::InteractionTool::onInitialize();

  this->enableMarkers(false);

  setName("Interact");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/Interact.png"));
}

void InteractiveTool::activate() {
  rviz_default_plugins::tools::InteractionTool::activate();

  this->enableMarkers(true);
}

void InteractiveTool::deactivate() {
  rviz_default_plugins::tools::InteractionTool::deactivate();

  this->enableMarkers(false);
}

}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(flatland_rviz_plugins::InteractiveTool, rviz_common::Tool)
