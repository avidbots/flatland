#ifndef FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_
#define FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_

#include <vector>
#include <rviz_default_plugins/tools/interaction/interaction_tool.hpp>
#include <rviz_common/display.hpp>

namespace flatland_rviz_plugins {
class RVIZ_DEFAULT_PLUGINS_PUBLIC InteractiveTool : public rviz_default_plugins::tools::InteractionTool {
 public:
  InteractiveTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

 private:
  void enableMarkers(bool is_enabled);
};
}

#endif  //FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_
