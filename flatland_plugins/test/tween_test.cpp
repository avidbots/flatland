/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  tween_test.cpp
 * @brief test tween test plugin
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

#include <flatland_plugins/tween.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class TweenPluginTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }

  static bool fltcmp(const float& n1, const float& n2, float epsilon = 1e-5) {
    if (std::isinf(n1) && std::isinf(n2)) {
      return true;
    }

    if (std::isnan(n1) && std::isnan(n2)) {
      return true;
    }

    bool ret = fabs(n1 - n2) < epsilon;
    return ret;
  }
};

/**
 * Test the tween plugin handles oneshot
 */
TEST_F(TweenPluginTest, once_test) {
  world_yaml = this_file_dir / fs::path("tween_tests/once.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  Tween* tween =
      dynamic_cast<Tween*>(w->plugin_manager_.model_plugins_[0].get());

  Body* b = tween->body_;

  // ONCE mode should start with triggered_ = true (running)
  ASSERT_TRUE(tween->triggered_);
  // No trigger_topic specified — subscriber should not be created
  ASSERT_FALSE(tween->trigger_sub_);

  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 2.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 1.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 2.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 2.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 1.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 3.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 4.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 2.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 3.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 4.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 2.0));

  delete w;
}

/**
 * Test that the tween plugin yoyos
 */
TEST_F(TweenPluginTest, yoyo_test) {
  world_yaml = this_file_dir / fs::path("tween_tests/yoyo.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  Tween* tween =
      dynamic_cast<Tween*>(w->plugin_manager_.model_plugins_[0].get());

  Body* b = tween->body_;

  // YOYO mode should start with triggered_ = true (running)
  ASSERT_TRUE(tween->triggered_);
  // No trigger_topic specified — subscriber should not be created
  ASSERT_FALSE(tween->trigger_sub_);

  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.5));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 10.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 10.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 1.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.5));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.5));
  w->Update(timekeeper);

  delete w;
}

/**
 * Test that the tween plugin loops
 */
TEST_F(TweenPluginTest, loop_test) {
  world_yaml = this_file_dir / fs::path("tween_tests/loop.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  Tween* tween =
      dynamic_cast<Tween*>(w->plugin_manager_.model_plugins_[0].get());

  Body* b = tween->body_;

  // LOOP mode should start with triggered_ = true (running)
  ASSERT_TRUE(tween->triggered_);
  // No trigger_topic specified — subscriber should not be created
  ASSERT_FALSE(tween->trigger_sub_);

  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 2.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 2.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.25));
  w->Update(timekeeper);
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 7.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 7.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.75));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 10));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 10));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 1.0));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 2.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 2.5));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.25));
  w->Update(timekeeper);
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 5.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 5.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.5));
  w->Update(timekeeper);

  delete w;
}

/**
 * Test that the tween plugin yoyo can be paused and resumed
 */
TEST_F(TweenPluginTest, yoyo_pause_test) {
  world_yaml = this_file_dir / fs::path("tween_tests/yoyo_pause.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  Tween* tween =
      dynamic_cast<Tween*>(w->plugin_manager_.model_plugins_[0].get());

  Body* b = tween->body_;

  // Start position
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x,  0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 0.0));
  w->Update(timekeeper);
  // After one step, should move forward
  float pos_x = b->physics_body_->GetPosition().x;
  float pos_y = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(fltcmp(pos_x, 5.0f));
  ASSERT_TRUE(fltcmp(pos_y, 5.0f));

  // Pause the tween by calling TriggerCallback directly
  std_msgs::Bool pause_msg;
  pause_msg.data = false;
  tween->TriggerCallback(pause_msg);

  w->Update(timekeeper);

  // Should stay at same position when paused
  float pos_x_paused = b->physics_body_->GetPosition().x;
  float pos_y_paused = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(fltcmp(pos_x_paused, 5.0f));
  ASSERT_TRUE(fltcmp(pos_y_paused, 5.0f));

  // Resume the tween
  std_msgs::Bool resume_msg;
  resume_msg.data = true;
  tween->TriggerCallback(resume_msg);

  w->Update(timekeeper);

  // Should continue moving
  float pos_x_resume = b->physics_body_->GetPosition().x;
  float pos_y_resume = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(fltcmp(pos_x_resume, 10.0f));
  ASSERT_TRUE(fltcmp(pos_y_resume, 10.0f));

  delete w;
}

/**
 * Test that the tween plugin loop can be paused and resumed
 */
TEST_F(TweenPluginTest, loop_pause_test) {
  world_yaml = this_file_dir / fs::path("tween_tests/loop_pause.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  Tween* tween =
      dynamic_cast<Tween*>(w->plugin_manager_.model_plugins_[0].get());

  Body* b = tween->body_;

  // Start position
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 0.0));
  w->Update(timekeeper);
  // After one step
  float pos_x = b->physics_body_->GetPosition().x;
  float pos_y = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(fltcmp(pos_x, 2.5f));
  ASSERT_TRUE(fltcmp(pos_y, 2.5f));

  // Pause the tween by calling TriggerCallback directly
  std_msgs::Bool pause_msg;
  pause_msg.data = false;
  tween->TriggerCallback(pause_msg);

  w->Update(timekeeper);

  // Should stay at same position when paused
  float pos_x_paused = b->physics_body_->GetPosition().x;
  float pos_y_paused = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(fltcmp(pos_x_paused, 2.5f));
  ASSERT_TRUE(fltcmp(pos_y_paused, 2.5f));

  // Resume the tween
  std_msgs::Bool resume_msg;
  resume_msg.data = true;
  tween->TriggerCallback(resume_msg);

  w->Update(timekeeper);

  // Should continue moving
  float pos_x_resume = b->physics_body_->GetPosition().x;
  float pos_y_resume = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(fltcmp(pos_x_resume, 5.0f));
  ASSERT_TRUE(fltcmp(pos_y_resume, 5.0f));

  delete w;
}

/**
 * Test that trigger mode defaults to backward direction (triggered_ = false)
 * and responds correctly to forward/backward triggers
 */
TEST_F(TweenPluginTest, trigger_default_test) {
  world_yaml = this_file_dir / fs::path("tween_tests/trigger.world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.5);
  World* w = World::MakeWorld(world_yaml.string());

  Tween* tween =
      dynamic_cast<Tween*>(w->plugin_manager_.model_plugins_[0].get());

  Body* b = tween->body_;

  // TRIGGER mode should start with triggered_ = false (backward direction)
  ASSERT_FALSE(tween->triggered_);

  // Start position
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().x, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetPosition().y, 0.0));
  ASSERT_TRUE(fltcmp(b->physics_body_->GetAngle(), 0.0));

  // With triggered_=false, the tween direction is backward so the body
  // stays near origin (tween steps forward then is set backward each cycle)
  w->Update(timekeeper);
  w->Update(timekeeper);
  // After a couple of updates with backward direction from start, stays near 0
  float pos_x = b->physics_body_->GetPosition().x;
  float pos_y = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(pos_x < 1.0f);  // Should not have moved significantly
  ASSERT_TRUE(pos_y < 1.0f);

  // Trigger forward
  std_msgs::Bool trigger_msg;
  trigger_msg.data = true;
  tween->TriggerCallback(trigger_msg);

  ASSERT_TRUE(tween->triggered_);

  // Now it should move forward
  w->Update(timekeeper);
  w->Update(timekeeper);
  pos_x = b->physics_body_->GetPosition().x;
  pos_y = b->physics_body_->GetPosition().y;
  ASSERT_TRUE(pos_x > 1.0f);  // Should have moved forward significantly

  // Trigger backward
  trigger_msg.data = false;
  tween->TriggerCallback(trigger_msg);

  ASSERT_FALSE(tween->triggered_);

  // First update applies the backward direction
  w->Update(timekeeper);
  // Store position after direction change has been applied
  float pos_before = b->physics_body_->GetPosition().x;
  w->Update(timekeeper);
  float pos_after = b->physics_body_->GetPosition().x;
  ASSERT_TRUE(pos_after <= pos_before);  // Moving backward or at boundary

  delete w;
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "tween_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}