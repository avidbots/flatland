#include <ros/ros.h>
#include <signal.h>

#include "flatland_server/simulation_manager.h"

/** Global variables */
flatland_server::SimulationManager* simulation_manager;

/**
 * @name        SigintHandler
 * @brief       Interrupt handler - sends shutdown signal to simulation_manager
 * @param[in]   sig: signal itself
 */
void SigintHandler(int sig)
{
  ROS_WARN_NAMED("flatland_server", "*** Shutting down... ***");

  if (simulation_manager != nullptr)
  {
    simulation_manager->Shutdown();
    delete simulation_manager;
    simulation_manager = nullptr;
  }
  ROS_INFO_STREAM_NAMED("flatland_server", "Beginning ros shutdown");
  ros::shutdown();
}

/**
 * @name        main
 * @brief       Entrypoint for Flatland Server ros node
 */
int main(int argc, char**argv)
{
  ros::init(argc, argv, "flatland_server", ros::init_options::NoSigintHandler);
  ros::NodeHandle node_handle;

  // Load parameters
  double initial_rate = 60.0;
  std::string world_path;
  if (node_handle.getParam("initial_rate", initial_rate)) {
    ROS_INFO_STREAM_NAMED("flatland_server", "initial rate: " << initial_rate);
  } else {
    ROS_INFO_STREAM_NAMED("flatland_server", "assuming initial rate: " << initial_rate);
  }

  if (node_handle.getParam("world_path", world_path)) {
    ROS_INFO_STREAM_NAMED("flatland_server", "world path: " << world_path);
  } else {
    ROS_FATAL_NAMED("flatland_server", "No world_path parameter given!");
    ros::shutdown();
    return 1;
  }

  // Create simulation manager object
  simulation_manager = new flatland_server::SimulationManager();

  // Register sigint shutdown handler
  signal(SIGINT, SigintHandler);

  ROS_INFO_STREAM_NAMED("flatland_server", "Initialized");
  simulation_manager->main();

  ROS_INFO_STREAM_NAMED("flatland_server", "Returned from simulation manager main");
  delete simulation_manager;
  simulation_manager = nullptr;
  return 0;
}
