/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  dummy_world_plugin_test.cpp
 * @brief test world plugin
 * @author Yi Ren
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

#include <flatland_server/dummy_world_plugin.h>
#include <flatland_server/world.h>
#include <flatland_server/world_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using namespace flatland_server;

TEST(DummyWorldPluginTest, pluginlib_load_test) {
  pluginlib::ClassLoader<flatland_server::WorldPlugin> loader(
      "flatland_server", "flatland_server::WorldPlugin");

  try {
    boost::shared_ptr<flatland_server::WorldPlugin> plugin =
        loader.createInstance("flatland_plugins::DummyWorldPlugin");

    YAML::Node n = YAML::Node();
    YamlReader reader = YamlReader();
    plugin->Initialize(NULL, "DummyWorldPluginName", "DummyWorldPluginType", n,
                       reader);
  } catch (pluginlib::PluginlibException& e) {
    FAIL() << "Failed to load Dummy World Plugin. " << e.what();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_world_plugin_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
