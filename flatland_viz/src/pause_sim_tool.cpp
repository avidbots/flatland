/*
 * @copyright Copyright 2022 FEUP
 * @name	pause_sim_tool.cpp
 * @brief pauses simulation
 * @author Ana Barros
 * @author Henrique Ribeiro
 * @author João Costa
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, FEUP
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

#include <flatland_viz/pause_sim_tool.h>

namespace flatland_viz
{

PauseSimTool::PauseSimTool() {}

// Disconnect the service client when the tool's destructor is called
PauseSimTool::~PauseSimTool() { rclcpp::shutdown(); /*pause_service_->shutdown();*/ }

// When the tool is initially loaded, connect to the pause toggle service
void PauseSimTool::onInitialize()
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pause_sim_tool");  // TODO
  pause_service_ = node->create_client<std_srvs::srv::Empty>("toggle_pause");
  setName("Pause/Resume");
}

// Every time the user presses the tool's Rviz toolbar button, call the pause
// toggle service
void PauseSimTool::activate()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = pause_service_->async_send_request(request);
}

void PauseSimTool::deactivate() {}

}  // end namespace flatland_viz

// Tell pluginlib about the tool class
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(flatland_viz::PauseSimTool, rviz_common::Tool)
