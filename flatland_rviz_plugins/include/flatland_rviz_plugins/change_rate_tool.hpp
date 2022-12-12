#ifndef FLATLAND_RVIZ_PLUGINS__CHANGE_RATE_TOOL_HPP_
#define FLATLAND_RVIZ_PLUGINS__CHANGE_RATE_TOOL_HPP_

#include <QObject>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <flatland_msgs/srv/change_rate.hpp>

#include "rviz_default_plugins/visibility_control.hpp"

namespace flatland_rviz_plugins {

class RVIZ_DEFAULT_PLUGINS_PUBLIC ChangeRateTool : public rviz_common::Tool {
  Q_OBJECT

 public:
  ChangeRateTool();
  ~ChangeRateTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  void setRate(double rate);

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<flatland_msgs::srv::ChangeRate>::SharedPtr change_rate_service_;
};

}  // namespace flatland_rviz_plugins

#endif  // FLATLAND_RVIZ_PLUGINS__CHANGE_RATE_TOOL_HPP_
