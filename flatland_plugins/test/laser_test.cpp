/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  laser_test.cpp
 * @brief test laser plugin
 * @author Chunshang Li
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

#include <flatland_plugins/laser.h>
#include <flatland_server/model_plugin.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserPluginTest : public ::testing::Test {
 protected:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  Timekeeper time_keeper;

  LaserPluginTest() {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }
};

TEST_F(LaserPluginTest, load_test) {
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
      "flatland_server", "flatland_server::ModelPlugin");

  try {
    boost::shared_ptr<flatland_server::ModelPlugin> laser =
        loader.createInstance("flatland_plugins::Laser");
  } catch (pluginlib::PluginlibException& e) {
    FAIL() << "Failed to load Laser plugin. " << e.what();
  }
}

TEST_F(LaserPluginTest, range_test) {
  world_yaml = this_file_dir / fs::path("laser_tests/range_test/world.yaml");
  time_keeper.SetMaxStepSize(1.0);
  World* w = World::MakeWorld(world_yaml.string());

  sensor_msgs scan_front, scan_center, scan_back;

  auto scan_front_callback = [scan_front](const sensor_msgs::LaserScan& msg) {
    scan_front = msg;
  };

  auto scan_center_callback = [scan_center](const sensor_msgs::LaserScan& msg) {
    scan_center = msg;
  };

  auto scan_back_callback = [scan_back](const sensor_msgs::LaserScan& msg) {
    scan_back = msg;
  };

  ros::NodeHandle nh;
  ros::Subscriber sub_1 = nh.subscribe("/scan_front", 1, scan_front_callback);
  ros::Subscriber sub_2 = nh.subscribe("/scan_center", 1, scan_center_callback);
  ros::Subscriber sub_3 = nh.subscribe("/scan_back", 1, scan_back_callback);

  w->Update(time_keeper);
  ros::SpinOnce();

  


}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
