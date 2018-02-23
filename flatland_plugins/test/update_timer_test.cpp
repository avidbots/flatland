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

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class TestPlugin : public ModelPlugin {
 public:
  UpdateTimer update_timer_;
  int update_counter_;

  void OnInitialize(const YAML::Node& config) override {
    update_timer_.SetRate(0);
    update_counter_ = 0;
  }

  void BeforePhysicsStep(const Timekeeper& timekeeper) override {
    // keeps this function updating at a specific rate
    if (!update_timer_.CheckUpdate(timekeeper)) {
      return;
    }
    update_counter_++;
  }
};

class UpdateTimerTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  double set_rate;
  double expected_rate;
  double actual_rate;
  double wall_rate;
  double step_size;
  double sim_test_time;
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

  void ExecuteRateTest() {
    Timekeeper timekeeper;
    w = World::MakeWorld(world_yaml.string());

    // artificially load a plugin
    boost::shared_ptr<TestPlugin> p(new TestPlugin());
    p->Initialize("TestPlugin", "test_plugin", w->models_[0], YAML::Node());
    w->plugin_manager_.model_plugins_.push_back(p);

    p->update_timer_.SetRate(set_rate);

    timekeeper.SetMaxStepSize(step_size);
    ros::WallRate rate(wall_rate);

    // run for two seconds
    while (timekeeper.GetSimTime() < ros::Time(sim_test_time)) {
      w->Update(timekeeper);
      ros::spinOnce();
      rate.sleep();
    }

    actual_rate = p->update_counter_ / timekeeper.GetSimTime().toSec();

    printf("Actual Rate: %f, Expected Rate: %f\n", actual_rate, expected_rate);
  }
};

/**
 * Test update rate at real time factor > 1
 */
TEST_F(UpdateTimerTest, rate_test_A) {
  world_yaml = this_file_dir / fs::path("update_timer_test/world.yaml");
  set_rate = 141.56;
  expected_rate = set_rate;
  sim_test_time = 2.0;

  // This makes the simulation run at 2.36936936937 real time speed
  wall_rate = 789.0;
  step_size = 1 / 333.0;

  ExecuteRateTest();

  EXPECT_NEAR(actual_rate, expected_rate, 1);
}

/**
 * Test update rate at real time factor < 1
 */
TEST_F(UpdateTimerTest, rate_test_B) {
  world_yaml = this_file_dir / fs::path("update_timer_test/world.yaml");
  set_rate = 564.56;
  expected_rate = set_rate;
  sim_test_time = 1.0;

  // This makes the simulation run at 0.179902755 real time speed
  wall_rate = 222.0;
  step_size = 1 / 1234.0;

  ExecuteRateTest();

  EXPECT_NEAR(actual_rate, expected_rate, 1);
}

/**
 * Test update rate at real time factor >> 1
 */
TEST_F(UpdateTimerTest, rate_test_C) {
  world_yaml = this_file_dir / fs::path("update_timer_test/world.yaml");
  set_rate = 47.4;
  expected_rate = set_rate;
  sim_test_time = 2;

  // This makes the simulation run at 10x real time speed
  wall_rate = 1000.0;
  step_size = 1 / 100.0;

  ExecuteRateTest();

  EXPECT_NEAR(actual_rate, expected_rate, 1);
}

/**
 * Test update rate at update rate = inf, which will update as fast as possible
 */
TEST_F(UpdateTimerTest, rate_test_D) {
  world_yaml = this_file_dir / fs::path("update_timer_test/world.yaml");
  set_rate = std::numeric_limits<double>::infinity();
  expected_rate = 100.0;
  sim_test_time = 2;

  // This makes the simulation run at 10x real time speed
  wall_rate = 1000.0;
  step_size = 1 / 100.0;

  ExecuteRateTest();

  EXPECT_NEAR(actual_rate, expected_rate, 1);
}

/**
 * Test update rate at update rate = 0, which will never update
 */
TEST_F(UpdateTimerTest, rate_test_E) {
  world_yaml = this_file_dir / fs::path("update_timer_test/world.yaml");
  set_rate = 0;
  expected_rate = 0;
  sim_test_time = 2;

  // This makes the simulation run at 10x real time speed
  wall_rate = 1000.0;
  step_size = 1 / 100.0;

  ExecuteRateTest();

  EXPECT_NEAR(actual_rate, expected_rate, 1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "model_tf_plugin_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
