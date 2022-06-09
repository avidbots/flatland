#ifndef PAUSE_SIM_TOOL_H
#define PAUSE_SIM_TOOL_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <std_srvs/srv/empty.hpp>

namespace flatland_viz {

/**
 * @name                PauseSimTool
 * @brief               Rviz tool to support pausing and unpausing the
 * simulation.
 */
class PauseSimTool : public rviz_common::Tool {
 public:
  PauseSimTool();
  ~PauseSimTool();

 private:
  /**
   * @name                onInitialize
   * @brief               Initializes tools currently loaded when rviz starts
   */
  virtual void onInitialize();

  /**
  virtual void activate();
   * @name                activate
   * @brief               Call the pause toggle service
   */
  virtual void activate();

  /**
   * @name                deactivate
   * @brief               Cleanup when tool is removed
   */
  virtual void deactivate();

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_service_;
};
}

#endif  // PAUSE_SIM_TOOL_H
