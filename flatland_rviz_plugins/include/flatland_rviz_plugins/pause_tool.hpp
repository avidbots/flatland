#ifndef FLATLAND_RVIZ_PLUGINS__PAUSE_TOOL_HPP_
#define FLATLAND_RVIZ_PLUGINS__PAUSE_TOOL_HPP_

#include <rviz_common/tool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <QObject>

#include <memory>

#include "rviz_default_plugins/visibility_control.hpp"

namespace flatland_rviz_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC PauseTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PauseTool();
  ~PauseTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_service_;
};

}  // namespace flatland_rviz_plugins

#endif  // FLATLAND_RVIZ_PLUGINS__PAUSE_TOOL_HPP_
