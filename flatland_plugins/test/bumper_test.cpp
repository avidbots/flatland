/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  bumper_test.cpp
 * @brief bumper laser plugin
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

#include <flatland_msgs/Collisions.h>
#include <flatland_plugins/bumper.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class BumperPluginTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  flatland_msgs::Collisions collision_msg;

  BumperPluginTest() {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }

  static bool fltcmp(const double& n1, const double& n2) {
    if (std::isinf(n1) && std::isinf(n2)) {
      return true;
    }

    if (std::isnan(n1) && std::isnan(n2)) {
      return true;
    }

    bool ret = fabs(n1 - n2) < 1e-5;
    return ret;
  }

  void CollisionCb(const flatland_msgs::Collisions& msg) {
    collision_msg = msg;
  };
};

/**
 * Test the bumper plugin for a given model and plugin configuration
 */
TEST_F(BumperPluginTest, collision_test) {
  world_yaml = this_file_dir / fs::path("bumper_test/collision_test/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(1.0);
  World* w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_1, sub_2, sub_3;
  LaserPluginTest* obj = dynamic_cast<LaserPluginTest*>(this);
  sub_1 = nh.subscribe("r/scan", 1, &LaserPluginTest::ScanFrontCb, obj);
  sub_2 = nh.subscribe("scan_center", 1, &LaserPluginTest::ScanCenterCb, obj);
  sub_3 = nh.subscribe("r/scan_back", 1, &LaserPluginTest::ScanBackCb, obj);

  Laser* p1 = dynamic_cast<Laser*>(w->plugin_manager_.model_plugins_[0].get());
  Laser* p2 = dynamic_cast<Laser*>(w->plugin_manager_.model_plugins_[1].get());
  Laser* p3 = dynamic_cast<Laser*>(w->plugin_manager_.model_plugins_[2].get());

  // let it spin for 10 times to make sure the message gets through
  ros::WallRate rate(500);
  for (int i = 0; i < 10; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
    rate.sleep();
  }

  // check scan returns
  EXPECT_TRUE(ScanEq(scan_front, "r/laser_front", -M_PI / 2, M_PI / 2, M_PI / 2,
                     0.0, 0.0, 0.0, 5.0, {4.5, 4.4, 4.3}, {}));
  EXPECT_TRUE(fltcmp(p1->update_rate_, std::numeric_limits<float>::infinity()))
      << "Actual: " << p1->update_rate_;
  EXPECT_EQ(p1->body_, w->models_[0]->bodies_[0]);

  EXPECT_TRUE(ScanEq(scan_center, "r/center_laser", 0, 2 * M_PI, M_PI / 2, 0.0,
                     0.0, 0.0, 5.0, {4.8, 4.7, 4.6, 4.9, 4.8}, {}));
  EXPECT_TRUE(fltcmp(p2->update_rate_, 5000)) << "Actual: " << p2->update_rate_;
  EXPECT_EQ(p2->body_, w->models_[0]->bodies_[0]);

  EXPECT_TRUE(ScanEq(scan_back, "r/laser_back", 0, 2 * M_PI, M_PI / 2, 0.0, 0.0,
                     0.0, 4, {NAN, 3.2, 3.5, NAN, NAN}, {}));
  EXPECT_TRUE(fltcmp(p3->update_rate_, 1)) << "Actual: " << p2->update_rate_;
  EXPECT_EQ(p3->body_, w->models_[0]->bodies_[0]);

  delete w;
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "bumper_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
