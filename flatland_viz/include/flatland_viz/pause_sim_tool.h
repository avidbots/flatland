#ifndef PAUSE_SIM_TOOL_H
#define PAUSE_SIM_TOOL_H

#include <ros/ros.h>
#include <rviz/tool.h>
#include <std_srvs/Empty.h>

namespace flatland_viz {

/**
 * @name                PauseSimTool
 * @brief               Rviz tool to support pausing and unpausing the
 * simulation.
 */
class PauseSimTool : public rviz::Tool {
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

  ros::NodeHandle nh_;  ///< NodeHandle to call the pause toggle service
  ros::ServiceClient
      pause_service_;  ///< ServiceClient that calls the pause toggle service
};
}

#endif  // PAUSE_SIM_TOOL_H
