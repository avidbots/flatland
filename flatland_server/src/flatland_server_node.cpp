/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	flatland_server_ndoe.cpp
 * @brief	Load params and run the ros node for flatland_server
 * @author Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <signal.h>
#include <string>

#include "flatland_server/simulation_manager.h"

/** Global variables */
flatland_server::SimulationManager *simulation_manager;

/**
 * @name        SigintHandler
 * @brief       Interrupt handler - sends shutdown signal to simulation_manager
 * @param[in]   sig: signal itself
 */
void SigintHandler(int sig) {
  ROS_WARN_NAMED("Node", "*** Shutting down... ***");

  if (simulation_manager != nullptr) {
    simulation_manager->Shutdown();
    delete simulation_manager;
    simulation_manager = nullptr;
  }
  ROS_INFO_STREAM_NAMED("Node", "Beginning ros shutdown");
  ros::shutdown();
}

/**
 * @name        main
 * @brief       Entrypoint for Flatland Server ros node
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "flatland", ros::init_options::NoSigintHandler);
  ros::NodeHandle node_handle("~");

  // Load parameters
  std::string world_path;  // The file path to the world.yaml file
  if (!node_handle.getParam("world_path", world_path)) {
    ROS_FATAL_NAMED("Node", "No world_path parameter given!");
    ros::shutdown();
    return 1;
  }

  float update_rate = 200.0;  // The physics update rate (Hz)
  node_handle.getParam("update_rate", update_rate);

  float step_size = 1 / 200.0;
  node_handle.getParam("step_size", step_size);

  bool show_viz = false;
  node_handle.getParam("show_viz", show_viz);

  float viz_pub_rate = 30.0;
  node_handle.getParam("viz_pub_rate", viz_pub_rate);

  // Create simulation manager object
  simulation_manager = new flatland_server::SimulationManager(
      world_path, update_rate, step_size, show_viz, viz_pub_rate);

  // Register sigint shutdown handler
  signal(SIGINT, SigintHandler);

  ROS_INFO_STREAM_NAMED("Node", "Initialized");
  simulation_manager->Main();

  ROS_INFO_STREAM_NAMED("Node", "Returned from simulation manager main");
  delete simulation_manager;
  simulation_manager = nullptr;
  return 0;
}
