/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   flatland_viz_node.cpp
 * @brief  The main ROS node for flatland_viz
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

#include <signal.h>
#include <iostream>

#include <QApplication>
#include <QSurfaceFormat>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

#include "flatland_viz/flatland_window.h"

FlatlandWindow * window = nullptr;

/**
 * @name        SigintHandler
 * @brief       Interrupt handler - sends shutdown signal to window and ROS
 * @param[in]   sig: signal itself
 */
void SigintHandler(int sig)
{
  RCLCPP_WARN(rclcpp::get_logger("Node"), "*** Shutting down... ***");

  if (window != nullptr) {
    // Window destructor will handle ROS client abstraction shutdown
    delete window;
    window = nullptr;
  }
  
  RCLCPP_INFO_STREAM(rclcpp::get_logger("Flatland Viz"), "Shutdown complete");
}

int main(int argc, char ** argv)
{
  // Check for display environment
  const char* display = getenv("DISPLAY");
  if (!display || strlen(display) == 0) {
    std::cerr << "Error: No DISPLAY environment variable set. Cannot run GUI application." << std::endl;
    return 1;
  }
  RCLCPP_INFO(rclcpp::get_logger("FlatlandVizNode"), "DISPLAY environment: %s", display);

  // Remove ROS arguments before passing to QApplication (following RViz2 pattern)
  std::vector<std::string> non_ros_args = rclcpp::remove_ros_arguments(argc, argv);

  // Convert to char* array for QApplication
  std::vector<char *> non_ros_args_c_strings;
  for (auto & arg : non_ros_args) {
    non_ros_args_c_strings.push_back(&arg.front());
  }
  int non_ros_argc = static_cast<int>(non_ros_args_c_strings.size());

  // Initialize QApplication with non-ROS arguments only
  QApplication app(non_ros_argc, non_ros_args_c_strings.data());
  
  // Debug Qt platform information
  RCLCPP_INFO(rclcpp::get_logger("FlatlandVizNode"), "Qt platform: %s", QApplication::platformName().toStdString().c_str());
  
  // Set Qt OpenGL format (defensive programming)
  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setVersion(2, 1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);
  QSurfaceFormat::setDefaultFormat(format);
  RCLCPP_INFO(rclcpp::get_logger("FlatlandVizNode"), "OpenGL format configured");

  RCLCPP_WARN(rclcpp::get_logger("FlatlandVizNode"), "Creating FlatlandWindow...");
  // Create window with ROS initialization deferred to FlatlandViz
  window = new FlatlandWindow(argc, argv);  // Pass original argc/argv for ROS init
  
  RCLCPP_WARN(rclcpp::get_logger("FlatlandVizNode"), "Showing window...");
  try {
    // Let the deferred initialization handle showing properly
    window->show();
    RCLCPP_WARN(rclcpp::get_logger("FlatlandVizNode"), "Window shown successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("FlatlandVizNode"), "Failed to show window: %s", e.what());
    delete window;
    window = nullptr;
    return 1;
  }

  RCLCPP_WARN(rclcpp::get_logger("FlatlandVizNode"), "Entering Qt event loop...");
  // Register sigint shutdown handler
  signal(SIGINT, SigintHandler);

  int result = app.exec();

  delete window;
  window = nullptr;
  return result;
}
