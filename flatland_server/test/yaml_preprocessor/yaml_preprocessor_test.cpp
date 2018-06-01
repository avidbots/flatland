/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	null.cpp
 * @brief	Sanity check / example test file
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

#include "flatland_server/yaml_preprocessor.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cmath>

namespace fs = boost::filesystem;
using namespace flatland_server;

void compareNodes(const char *p1, YAML::Node &a, YAML::Node &b) {
  try {
    EXPECT_STREQ(b[p1].as<std::string>().c_str(),
                 a[p1].as<std::string>().c_str())
        << "at " << p1;
  } catch (...) {
    ADD_FAILURE() << "Failure to compare " << p1;
  }
}

void compareNodes(const char *p1, int p2, YAML::Node &a, YAML::Node &b) {
  try {
    EXPECT_STREQ(b[p1][p2].as<std::string>().c_str(),
                 a[p1][p2].as<std::string>().c_str())
        << "at " << p1 << ":" << p2;
  } catch (...) {
    ADD_FAILURE() << "Failure to compare " << p1 << ":" << p2;
  }
}

void compareNodes(const char *p1, const char *p2, YAML::Node &a,
                  YAML::Node &b) {
  try {
    EXPECT_STREQ(b[p1][p2].as<std::string>().c_str(),
                 a[p1][p2].as<std::string>().c_str())
        << "at " << p1 << ":" << p2;
  } catch (...) {
    ADD_FAILURE() << "Failure to compare " << p1 << ":" << p2;
  }
}

// Test the bodyToMarkers method on a polygon shape
TEST(YamlPreprocTest, testEvalStrings) {
  boost::filesystem::path cwd = fs::path(__FILE__).parent_path();

  YAML::Node in = YamlPreprocessor::LoadParse(
      (cwd / fs::path("/yaml/eval.strings.yaml")).string());

  YAML::Node out = YamlPreprocessor::LoadParse(
      (cwd / fs::path("/yaml/eval.strings.out.yaml")).string());

  // check that the two strings match
  compareNodes("foo", in, out);
  compareNodes("bar", in, out);
  compareNodes("baz", in, out);
  compareNodes("bash", in, out);

  compareNodes("boop", "bal", in, out);

  compareNodes("foop", 0, in, out);
  compareNodes("foop", 1, in, out);
  compareNodes("foop", 2, in, out);

  compareNodes("bom", in, out);

  compareNodes("testEnv", "env1", in, out);
  compareNodes("testEnv", "env2", in, out);
  compareNodes("testEnv", "env3", in, out);
  compareNodes("testEnv", "env4", in, out);
  compareNodes("testEnv", "env5", in, out);
  compareNodes("testEnv", "env6", in, out);
  compareNodes("testEnv", "env7", in, out);
  compareNodes("testEnv", "env8", in, out);

  compareNodes("testParam", "param1", in, out);
  compareNodes("testParam", "param2", in, out);
  compareNodes("testParam", "param3", in, out);
  compareNodes("testParam", "param4", in, out);
  compareNodes("testParam", "param5", in, out);
  compareNodes("testParam", "param6", in, out);
  compareNodes("testParam", "param7", in, out);
  compareNodes("testParam", "param8", in, out);
  compareNodes("testParam", "param9", in, out);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ros::init(argc, argv, "yaml_preprocessor_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
