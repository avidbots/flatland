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
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <sensor_msgs/LaserScan.h>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class LaserPluginTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  sensor_msgs::LaserScan scan_front, scan_center, scan_back;
  World* w;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    w = nullptr;
  }

  void TearDown() override {
    if (w != nullptr) {
      delete w;
    }
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

  // print content of the vector for debugging
  void print_flt_vec(const std::vector<float>& v) {
    printf("{");
    for (const auto& e : v) {
      printf("%f,", e);
    }
    printf("}");
  }

  // check the float values equals and print message for debugging
  bool FloatEq(const char* name, float actual, float expected) {
    if (actual != expected) {
      printf("%s Actual:%f != Expected %f", name, actual, expected);
      return false;
    }
    return true;
  }

  // check the received scan data is as expected
  bool ScanEq(const sensor_msgs::LaserScan& scan, std::string frame_id,
              float angle_min, float angle_max, float angle_increment,
              float time_increment, float scan_time, float range_min,
              float range_max, std::vector<float> ranges,
              std::vector<float> intensities) {
    if (scan.header.frame_id != frame_id) {
      printf("frame_id Actual:%s != Expected:%s\n",
             scan.header.frame_id.c_str(), frame_id.c_str());
      return false;
    }

    if (!FloatEq("angle_min", scan.angle_min, angle_min)) return false;
    if (!FloatEq("angle_max", scan.angle_max, angle_max)) return false;
    if (!FloatEq("angle_increment", scan.angle_increment, angle_increment))
      return false;
    if (!FloatEq("time_increment", scan.time_increment, time_increment))
      return false;
    if (!FloatEq("scan_time", scan.scan_time, scan_time)) return false;
    if (!FloatEq("range_min", scan.range_min, range_min)) return false;
    if (!FloatEq("range_max", scan.range_max, range_max)) return false;

    if (ranges.size() != scan.ranges.size() ||
        !std::equal(ranges.begin(), ranges.end(), scan.ranges.begin(),
                    fltcmp)) {
      printf("\"ranges\" does not match\n");
      printf("Actual: ");
      print_flt_vec(scan.ranges);
      printf("\n");
      printf("Expected: ");
      print_flt_vec(ranges);
      printf("\n");
      return false;
    }

    if (intensities.size() != scan.intensities.size() ||
        !std::equal(intensities.begin(), intensities.end(),
                    scan.intensities.begin(), fltcmp)) {
      printf("\"intensities\" does not math");
      printf("Actual: ");
      print_flt_vec(scan.intensities);
      printf("\n");
      printf("Expected: ");
      print_flt_vec(intensities);
      printf("\n");
      return false;
    }

    return true;
  }

  void ScanFrontCb(const sensor_msgs::LaserScan& msg) { scan_front = msg; };
  void ScanCenterCb(const sensor_msgs::LaserScan& msg) { scan_center = msg; };
  void ScanBackCb(const sensor_msgs::LaserScan& msg) { scan_back = msg; };
};

/**
 * Test the laser plugin for a given model and plugin configuration
 */
TEST_F(LaserPluginTest, range_test) {
  world_yaml = this_file_dir / fs::path("laser_tests/range_test/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(1.0);
  w = World::MakeWorld(world_yaml.string());

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
  for (unsigned int i = 0; i < 10; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
    rate.sleep();
  }

  // check scan returns
  EXPECT_TRUE(ScanEq(scan_front, "r_laser_front", -M_PI / 2, M_PI / 2, M_PI / 2,
                     0.0, 0.0, 0.0, 5.0, {4.5, 4.4, 4.3}, {}));
  EXPECT_TRUE(fltcmp(p1->update_rate_, std::numeric_limits<float>::infinity()))
      << "Actual: " << p1->update_rate_;
  EXPECT_EQ(p1->body_, w->models_[0]->bodies_[0]);

  EXPECT_TRUE(ScanEq(scan_center, "r_center_laser", 0, 2 * M_PI, M_PI / 2, 0.0,
                     0.0, 0.0, 5.0, {4.8, 4.7, 4.6, 4.9, 4.8}, {}));
  EXPECT_TRUE(fltcmp(p2->update_rate_, 5000)) << "Actual: " << p2->update_rate_;
  EXPECT_EQ(p2->body_, w->models_[0]->bodies_[0]);

  EXPECT_TRUE(ScanEq(scan_back, "r_laser_back", 0, 2 * M_PI, M_PI / 2, 0.0, 0.0,
                     0.0, 4, {NAN, 3.2, 3.5, NAN, NAN}, {}));
  EXPECT_TRUE(fltcmp(p3->update_rate_, 1)) << "Actual: " << p2->update_rate_;
  EXPECT_EQ(p3->body_, w->models_[0]->bodies_[0]);
}
/**
 * Test the laser plugin for intensity configuration
 */
TEST_F(LaserPluginTest, intensity_test) {
  world_yaml =
      this_file_dir / fs::path("laser_tests/intensity_test/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(1.0);
  w = World::MakeWorld(world_yaml.string());

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
  for (unsigned int i = 0; i < 10; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
    rate.sleep();
  }

  // check scan returns
  EXPECT_TRUE(ScanEq(scan_front, "r_laser_front", -M_PI / 2, M_PI / 2, M_PI / 2,
                     0.0, 0.0, 0.0, 5.0, {4.5, 4.4, 4.3}, {0, 0, 0}));
  EXPECT_TRUE(fltcmp(p1->update_rate_, std::numeric_limits<float>::infinity()))
      << "Actual: " << p1->update_rate_;
  EXPECT_EQ(p1->body_, w->models_[0]->bodies_[0]);
  EXPECT_TRUE(ScanEq(scan_center, "r_center_laser", 0, 2 * M_PI, M_PI / 2, 0.0,
                     0.0, 0.0, 5.0, {4.8, 4.7, 4.6, 4.9, 4.8},
                     {0, 255, 0, 0, 0}));
  EXPECT_TRUE(fltcmp(p2->update_rate_, 5000)) << "Actual: " << p2->update_rate_;
  EXPECT_EQ(p2->body_, w->models_[0]->bodies_[0]);
  EXPECT_TRUE(ScanEq(scan_back, "r_laser_back", 0, 2 * M_PI, M_PI / 2, 0.0, 0.0,
                     0.0, 4, {NAN, 3.2, 3.5, NAN, NAN}, {0, 0, 0, 0, 0}));
  EXPECT_TRUE(fltcmp(p3->update_rate_, 1)) << "Actual: " << p2->update_rate_;
  EXPECT_EQ(p3->body_, w->models_[0]->bodies_[0]);
}

/**
 * Checks the laser plugin will throw correct exception for invalid
 * configurations
 */
TEST_F(LaserPluginTest, invalid_A) {
  world_yaml = this_file_dir / fs::path("laser_tests/invalid_A/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());

    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException& e) {
    std::cmatch match;
    std::string regex_str = ".*Flatland YAML: Entry \"range\" does not exist.*";
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
        << "Exception Message '" + std::string(e.what()) + "'" +
               " did not match against regex '" + regex_str + "'";
  } catch (const std::exception& e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

/**
 * Checks the laser plugin will throw correct exception for invalid
 * configurations
 */
TEST_F(LaserPluginTest, invalid_B) {
  world_yaml = this_file_dir / fs::path("laser_tests/invalid_B/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());

    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException& e) {
    std::cmatch match;
    std::string regex_str = ".*Invalid \"angle\" params, must have max > min.*";
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
        << "Exception Message '" + std::string(e.what()) + "'" +
               " did not match against regex '" + regex_str + "'";
  } catch (const std::exception& e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
