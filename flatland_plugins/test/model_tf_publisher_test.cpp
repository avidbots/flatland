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
#include <pluginlib/class_loader.h>
#include <sensor_msgs/LaserScan.h>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class ModelTfPublisherTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;

  ModelTfPublisherTest() {
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

  void print_flt_vec(const std::vector<float>& v) {
    printf("{");
    for (const auto& e : v) {
      printf("%f,", e);
    }
    printf("}");
  }

  bool FloatEq(const char* name, float actual, float expected) {
    if (actual != expected) {
      printf("%s Actual:%f != Expected %f", name, actual, expected);
      return false;
    }
    return true;
  }

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
};

TEST_F(LaserPluginTest, tf_publish_test) {
  world_yaml = this_file_dir /
               fs::path("model_tf_publisher_test.cpp/tf_publish_test/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(1.0);
  World* w = World::MakeWorld(world_yaml.string());

  
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "model_tf_plugin_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
