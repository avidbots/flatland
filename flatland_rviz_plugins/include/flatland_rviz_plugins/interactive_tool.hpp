#ifndef FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_
#define FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_

#include <rviz_common/display.hpp>
#include <rviz_default_plugins/tools/interaction/interaction_tool.hpp>
#include <vector>

namespace flatland_rviz_plugins {
class RVIZ_DEFAULT_PLUGINS_PUBLIC InteractiveTool
    : public rviz_default_plugins::tools::InteractionTool {
 public:
  InteractiveTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

 private:
  void enableMarkers(bool is_enabled);
};
}  // namespace flatland_rviz_plugins

#endif  // FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_
