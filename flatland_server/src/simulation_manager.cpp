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

#include <flatland_server/debug_visualization.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <flatland_server/service_manager.h>
#include <flatland_server/world.h>

#include <cmath>
#include <exception>
#include <limits>
#include <string>

namespace flatland_server
{

SimulationManager::SimulationManager(
  std::shared_ptr<rclcpp::Node> node, std::string world_yaml_file, double update_rate,
  double step_size, bool show_viz, double viz_pub_rate)
: node_(node),
  world_(nullptr),
  update_rate_(update_rate),
  step_size_(step_size),
  show_viz_(show_viz),
  viz_pub_rate_(viz_pub_rate),
  world_yaml_file_(world_yaml_file),
  rate_(new rclcpp::WallRate(update_rate))
{
  RCLCPP_INFO(
    rclcpp::get_logger("SimMan"),
    "Simulation params: world_yaml_file(%s) update_rate(%f), "
    "step_size(%f) show_viz(%s), viz_pub_rate(%f)",
    world_yaml_file_.c_str(), update_rate_, step_size_, show_viz_ ? "true" : "false",
    viz_pub_rate_);
}

SimulationManager::~SimulationManager() { delete rate_; }

void SimulationManager::setUpdateRate(double update_rate)
{
  update_rate_ = update_rate;
  delete rate_;
  rate_ = new rclcpp::WallRate(update_rate_);
}

void SimulationManager::Main()
{
  RCLCPP_INFO(rclcpp::get_logger("SimMan"), "Initializing...");
  run_simulator_ = true;

  try {
    world_ = World::MakeWorld(node_, world_yaml_file_);
    RCLCPP_INFO(rclcpp::get_logger("SimMan"), "World loaded");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("SimMan"), "%s", e.what());
    return;
  }

  if (show_viz_) world_->DebugVisualize();

  int iterations = 0;
  double filtered_cycle_util = 0;
  double min_cycle_util = std::numeric_limits<double>::infinity();
  double max_cycle_util = 0;
  double viz_update_period = 1.0f / viz_pub_rate_;
  ServiceManager service_manager(this, world_);
  Timekeeper timekeeper(node_);

  rclcpp::Clock wall_clock(RCL_STEADY_TIME);
  timekeeper.SetMaxStepSize(step_size_);
  RCLCPP_INFO(rclcpp::get_logger("SimMan"), "Simulation loop started");

  while (rclcpp::ok() && run_simulator_) {
    // for updating visualization at a given rate
    // see flatland_plugins/update_timer.cpp for this formula
    double start_time = wall_clock.now().seconds();
    double f = 0.0;
    try {
      f = std::fmod(wall_clock.now().seconds() + (update_rate_ / 2.0), viz_update_period);
    } catch (std::runtime_error & ex) {
      RCLCPP_ERROR(rclcpp::get_logger("SimMan"), "Flatland runtime error: [%s]", ex.what());
    }
    bool update_viz = ((f >= 0.0) && (f < 1.0f / update_rate_));

    world_->Update(timekeeper);  // Step physics by ros cycle time

    if (show_viz_ && update_viz) {
      world_->DebugVisualize(false);                        // no need to update layer
      DebugVisualization::Get(node_)->Publish(timekeeper);  // publish debug visualization
    }

    rclcpp::spin_some(node_);
    double cycle_time = wall_clock.now().seconds() - start_time;
    rate_->sleep();

    iterations++;

    double expected_cycle_time = 1.0f / update_rate_;
    double cycle_util = cycle_time / expected_cycle_time * 100;  // in percent
    double factor = timekeeper.GetStepSize() / expected_cycle_time;
    min_cycle_util = std::min(cycle_util, min_cycle_util);
    if (iterations > 10) max_cycle_util = std::max(cycle_util, max_cycle_util);
    filtered_cycle_util = 0.99 * filtered_cycle_util + 0.01 * cycle_util;

    RCLCPP_INFO_THROTTLE(
      rclcpp::get_logger("SimMan"), wall_clock, 1000,
      "utilization: min %.1f%% max %.1f%% ave %.1f%%  factor: %.1f", min_cycle_util, max_cycle_util,
      filtered_cycle_util, factor);
  }
  RCLCPP_INFO(rclcpp::get_logger("SimMan"), "Simulation loop ended");

  delete world_;
}

void SimulationManager::Shutdown()
{
  RCLCPP_INFO(rclcpp::get_logger("SimMan"), "Shutdown called");
  run_simulator_ = false;
}

}  // namespace flatland_server
