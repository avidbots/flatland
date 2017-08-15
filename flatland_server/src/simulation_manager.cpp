/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	simulation_manager.cpp
 * @brief	Simulation manager runs the physics+event loop
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

#include "flatland_server/simulation_manager.h"
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <flatland_server/world.h>
#include <ros/ros.h>
#include <exception>
#include <string>
#include "flatland_server/debug_visualization.h"

namespace flatland_server {

SimulationManager::SimulationManager(std::string world_yaml_file,
                                     float initial_rate, bool show_viz,
                                     float viz_pub_rate)
    : initial_rate_(initial_rate),
      world_yaml_file_(world_yaml_file),
      show_viz_(show_viz),
      viz_pub_rate_(viz_pub_rate) {
  // Todo: Initialize SimTime class here once written

  ROS_INFO_NAMED("SimMan",
                 "Simulation params: world_yaml_file(%s) initial_rate(%f), "
                 "show_viz(%s), viz_pub_rate(%f)",
                 world_yaml_file_.c_str(), initial_rate_,
                 show_viz_ ? "true" : "false", viz_pub_rate_);
}

void SimulationManager::Main() {
  ROS_INFO_NAMED("SimMan", "Initializing...");

  try {
    world_ = World::MakeWorld(world_yaml_file_);
  } catch (const std::exception &e) {
    ROS_FATAL_NAMED("SimMan", "%s", e.what());
    return;
  }

  ROS_INFO_NAMED("SimMan", "World loaded");

  if (show_viz_) {
    world_->DebugVisualize();
  }

  timekeeper_.SetMaxStepSize(1.0 / initial_rate_);

  double filtered_cycle_utilization = 0;

  double viz_update_period = 1.0f / viz_pub_rate_;

  // TODO (Chunshang): Not sure how to do time so the faster than realtime
  // simulation can be done properly
  ros::WallRate rate(1.0 / timekeeper_.GetStepSize());
  ROS_INFO_NAMED("SimMan", "Simulation loop started");

  while (ros::ok() && run_simulator_) {
    // for updating visualization at a given rate, see
    // flatland_plugins/update_timer.cpp for this formula
    double f = fmod(
        ros::WallTime::now().toSec() + (rate.expectedCycleTime().toSec() / 2.0),
        viz_update_period);
    bool update_viz = ((f >= 0.0) && (f < rate.expectedCycleTime().toSec()));

    // Step physics by ros cycle time
    world_->Update(timekeeper_);

    if (show_viz_ && update_viz) {
      // don't update layers because they don't change
      world_->DebugVisualize(false);        //
      DebugVisualization::Get().Publish();  // publish debug visualization
    }

    ros::spinOnce();  // Normal ROS event loop
    rate.sleep();

    double cycle_utilization =
        rate.cycleTime().toSec() / rate.expectedCycleTime().toSec();

    filtered_cycle_utilization =
        0.99 * filtered_cycle_utilization + 0.01 * cycle_utilization;

    ROS_INFO_THROTTLE_NAMED(
        1.0, "SimMan", "cycle time %.2f/%.2fms (%.1f%%, %.1f%% average)",
        rate.cycleTime().toSec() * 1000,
        rate.expectedCycleTime().toSec() * 1000.0, 100.0 * cycle_utilization,
        100.0 * filtered_cycle_utilization);
  }

  ROS_INFO_NAMED("SimMan", "Simulation loop ended");
}

void SimulationManager::Shutdown() {
  ROS_INFO_NAMED("SimMan", "Shutdown called");
  delete world_;
}

};  // namespace flatland_server
