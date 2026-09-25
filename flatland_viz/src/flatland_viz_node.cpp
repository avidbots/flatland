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

#include <QApplication>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <string>
#include <vector>

#include "flatland_viz/flatland_viz.h"

int main(int argc, char ** argv)
{
  std::vector<std::string> qt_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  // Ogre's render window needs X11; use XWayland unless the user chose a platform (same as rviz2)
  if (
    qEnvironmentVariable("XDG_SESSION_TYPE") == "wayland" &&
    !qEnvironmentVariableIsSet("QT_QPA_PLATFORM")) {
    qputenv("QT_QPA_PLATFORM", "xcb");
  }

  std::vector<char *> qt_argv;
  for (auto & arg : qt_args) {
    qt_argv.push_back(arg.data());
  }
  int qt_argc = static_cast<int>(qt_argv.size());
  QApplication app(qt_argc, qt_argv.data());

  // rviz defaults to printing every log level to the console; route it through rclcpp instead
  auto logger = rclcpp::get_logger("flatland_viz");
  rviz_common::set_logging_handlers(
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_DEBUG(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_INFO(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_WARN(logger, "%s", msg.c_str());
    },
    [logger](const std::string & msg, const std::string &, size_t) {
      RCLCPP_ERROR(logger, "%s", msg.c_str());
    });
  rviz_common::install_rviz_rendering_log_handlers();

  auto rviz_ros_node =
    std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("flatland_viz");

  int ret;
  {
    FlatlandViz viz(rviz_ros_node);
    viz.show();

    // rclcpp's SIGINT handler only flips rclcpp::ok(), so poll it to exit cleanly
    QTimer shutdown_timer;
    QObject::connect(&shutdown_timer, &QTimer::timeout, [&app]() {
      if (!rclcpp::ok()) {
        app.quit();
      }
    });
    shutdown_timer.start(100);

    ret = app.exec();
  }

  rclcpp::shutdown();
  return ret;
}
