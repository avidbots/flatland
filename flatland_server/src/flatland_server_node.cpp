/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2020 Avidbots Corp.
 * @name	flatland_server_node.cpp
 * @brief	Load params and run the ros node for flatland_server
 * @author Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Avidbots Corp.
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

#include <signal.h>

#include <string>

#include "flatland_server/simulation_manager.h"
#include "rclcpp/rclcpp.hpp"

/** Global variables */
//

/**
 * @name        SigintHandler
 * @brief       Interrupt handler - sends shutdown signal to simulation_manager
 * @param[in]   sig: signal itself
 */
// void SigintHandler(int sig) {
//   RCLCPP_WARN(rclcpp::get_logger("Node"), "*** Shutting down... ***");

//   if (simulation_manager != nullptr) {
//     simulation_manager->Shutdown();
//     delete simulation_manager;
//     simulation_manager = nullptr;
//   }
//   RCLCPP_INFO_STREAM_NAMED("Node", "Beginning ros shutdown");
//   rclcpp::shutdown();
// }

class FlatlandServerNode : public rclcpp::Node
{
public:
  FlatlandServerNode() : Node("flatland_server")
  {
    declare_parameter<std::string>("world_path");
    declare_parameter<float>("update_rate");
    declare_parameter<float>("step_size");
    declare_parameter<bool>("show_viz");
    declare_parameter<float>("viz_pub_rate");

    // Load parameters
    if (!get_parameter("world_path", world_path_)) {
      RCLCPP_INFO(get_logger(), "No world_path parameter given!");
      rclcpp::shutdown();
      return;
    }
    get_parameter_or<float>("update_rate", update_rate_, 200.0f);
    get_parameter_or<float>("step_size", step_size_, 1.0f / 200.0f);
    get_parameter_or<bool>("show_viz", show_viz_, false);
    get_parameter_or<float>("viz_pub_rate", viz_pub_rate_, 30.0f);
  }

  void Run()
  {
    // Create simulation manager object
    simulation_manager_ = std::make_shared<flatland_server::SimulationManager>(
      shared_from_this(), world_path_, update_rate_, step_size_, show_viz_, viz_pub_rate_);

    RCLCPP_INFO(this->get_logger(), "Initialized");
    simulation_manager_->Main();

    RCLCPP_INFO(this->get_logger(), "Returned from simulation manager main2");
  }

  // TODO: Allow updates to step size, update rate etc. with new ros2 dynamic
  // params

private:
  std::string world_path_;  // The file path to the world.yaml file
  float update_rate_;       // The physics update rate (Hz)
  float step_size_;
  bool show_viz_;
  float viz_pub_rate_;
  std::shared_ptr<flatland_server::SimulationManager> simulation_manager_;
};

/**
 * @name        main
 * @brief       Entrypoint for Flatland Server ros node
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto flatland_server = std::make_shared<FlatlandServerNode>();
  flatland_server->Run();
  rclcpp::shutdown();
  return 0;
}
