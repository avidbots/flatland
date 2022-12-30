/*
 * @copyright Copyright 2022 FEUP
 * @name	pause_sim_tool.h
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

#ifndef PAUSE_SIM_TOOL_H
#define PAUSE_SIM_TOOL_H

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>
#include <std_srvs/srv/empty.hpp>

namespace flatland_viz
{

/**
 * @name                PauseSimTool
 * @brief               Rviz tool to support pausing and unpausing the
 * simulation.
 */
class PauseSimTool : public rviz_common::Tool
{
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
}  // namespace flatland_viz

#endif  // PAUSE_SIM_TOOL_H
