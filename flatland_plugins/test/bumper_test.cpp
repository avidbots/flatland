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
 * @brief test bumper plugin
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
#include <flatland_server/debug_visualization.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;
using namespace flatland_msgs;

class BumperPluginTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  flatland_msgs::Collisions msg1, msg2;
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

  bool StringEq(const std::string& name, const std::string& actual,
                const std::string& expected) {
    if (actual != expected) {
      printf("%s Actual:%s != Expected:%s\n", name.c_str(), actual.c_str(),
             expected.c_str());
      return false;
    }
    return true;
  }

  bool Vector2Eq(const std::string& name, const Vector2& actual,
                 const std::pair<float, float> expected) {
    if (!fltcmp(actual.x, expected.first) ||
        !fltcmp(actual.y, expected.second)) {
      printf("%s Actual:(%f,%f) != Expected:(%f,%f)\n", name.c_str(), actual.x,
             actual.y, expected.first, expected.second);
      return false;
    }
    return true;
  }

  bool CollisionsEq(const Collisions& collisions, const std::string& frame_id,
                    int num_collisions) {
    if (!StringEq("frame_id", collisions.header.frame_id, frame_id))
      return false;

    if (num_collisions != collisions.collisions.size()) {
      printf("Num collisions Actual:%lu != Expected:%d\n",
             collisions.collisions.size(), num_collisions);
      return false;
    }

    return true;
  }

  // check the received scan data is as expected
  bool CollisionEq(const Collision& collision, const std::string& entity_A,
                   const std::string& body_A, const std::string& entity_B,
                   const std::string& body_B, int return_size,
                   const std::pair<float, float>& normal) {
    if (!StringEq("entity_A", collision.entity_A, entity_A)) return false;
    if (!StringEq("body_A", collision.body_A, body_A)) return false;
    if (!StringEq("entity_B", collision.entity_B, entity_B)) return false;
    if (!StringEq("body_B", collision.body_B, body_B)) return false;

    if (!(collision.magnitude_forces.size() <= 2 &&
          collision.contact_positions.size() <= 2 &&
          collision.contact_normals.size() <= 2 &&
          collision.magnitude_forces.size() == return_size &&
          collision.contact_positions.size() == return_size &&
          collision.contact_normals.size() == return_size)) {
      printf(
          "Vector sizes are expected to be all the same and have sizes %d, "
          "magnitude_forces=%lu contact_positions=%lu contact_normals=%lu\n",
          return_size, collision.magnitude_forces.size(),
          collision.contact_positions.size(), collision.contact_normals.size());
      return false;
    }

    for (unsigned int i = 0; i < return_size; i++) {
      std::string idx = "[" + std::to_string(i) + "]";

      if (collision.magnitude_forces[i] <= 0) {
        printf("forces%s Actual:%f != Expected: >0\n", idx.c_str(),
               collision.magnitude_forces[i]);
        return false;
      }

      if (!Vector2Eq("normals" + idx, collision.contact_normals[i], normal)) {
        return false;
      }
    }
    return true;
  }

  void CollisionCb_A(const flatland_msgs::Collisions& msg) { msg1 = msg; }

  void CollisionCb_B(const flatland_msgs::Collisions& msg) { msg2 = msg; }

  void SpinRos(float hz, int iterations) {
    ros::WallRate rate(hz);
    for (unsigned int i = 0; i < iterations; i++) {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

/**
 * Test the bumper plugin for a given model and plugin configuration
 */
TEST_F(BumperPluginTest, collision_test) {
  world_yaml =
      this_file_dir / fs::path("bumper_tests/collision_test/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_1, sub_2, sub_3;
  BumperPluginTest* obj = dynamic_cast<BumperPluginTest*>(this);
  sub_1 = nh.subscribe("collisions", 1, &BumperPluginTest::CollisionCb_A, obj);
  sub_2 =
      nh.subscribe("collisions_B", 1, &BumperPluginTest::CollisionCb_B, obj);

  Bumper* p = dynamic_cast<Bumper*>(w->plugin_manager_.model_plugins_[0].get());

  Body* b0 = p->GetModel()->bodies_[0];
  Body* b1 = p->GetModel()->bodies_[1];

  // check that there are no collision at the begining
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  SpinRos(500, 10);  // make sure the messages gets through

  // check time is not zero to make sure message is received
  ASSERT_NE(msg1.header.stamp, ros::Time(0, 0));
  ASSERT_NE(msg2.header.stamp, ros::Time(0, 0));

  // step 15 time which makes the body move 1.5 meters, will make base_link_1
  // collide, but not base_link_2, not that base_link_1's fixture is a sensor
  for (unsigned int i = 0; i < 150; i++) {
    // Box2D needs velocity to be set every time step to ensure things are
    // moving at the desired velocity
    b0->physics_body_->SetLinearVelocity(b2Vec2(1, 0.0));
    w->Update(timekeeper);
    ros::spinOnce();
  }
  SpinRos(500, 10);  // makes sure the ros message gets through

  ASSERT_TRUE(CollisionsEq(msg1, "map", 1));
  EXPECT_TRUE(CollisionEq(msg1.collisions[0], "robot1", "base_link_1",
                          "layer_1", "layer_1", 0, {}));
  EXPECT_TRUE(CollisionsEq(msg2, "world", 0));

  // step another 5 times which moves 0.5 meters colliding base_link_2 as well
  for (unsigned int i = 0; i < 50; i++) {
    b0->physics_body_->SetLinearVelocity(b2Vec2(1, 0.0));
    w->Update(timekeeper);
    ros::spinOnce();
  }
  SpinRos(500, 10);
  ASSERT_TRUE(CollisionsEq(msg1, "map", 2));
  EXPECT_TRUE(CollisionEq(msg1.collisions[0], "robot1", "base_link_1",
                          "layer_1", "layer_1", 0, {}));
  EXPECT_TRUE(CollisionEq(msg1.collisions[1], "robot1", "base_link_2",
                          "layer_1", "layer_1", 1, {1, 0}));
  ASSERT_TRUE(CollisionsEq(msg2, "world", 1));
  EXPECT_TRUE(CollisionEq(msg2.collisions[0], "robot1", "base_link_2",
                          "layer_1", "layer_1", 1, {1, 0}));

  // Now move backward far away from the wall, there collisions should clear
  for (unsigned int i = 0; i < 300; i++) {
    b0->physics_body_->SetLinearVelocity(b2Vec2(-1, 0.0));
    w->Update(timekeeper);
    ros::spinOnce();
  }
  SpinRos(500, 10);

  EXPECT_TRUE(CollisionsEq(msg1, "map", 0));
  EXPECT_TRUE(CollisionsEq(msg2, "world", 0));

  // Teleport the body to the other side of the wall, try hitting the wall from
  // the other direction, the collision normal vector should be flipped now
  b0->physics_body_->SetTransform(b2Vec2(4, 0), 0);
  b1->physics_body_->SetTransform(b2Vec2(4, 0), 0);

  for (unsigned int i = 0; i < 300; i++) {
    b0->physics_body_->SetLinearVelocity(b2Vec2(-1, 0.0));
    w->Update(timekeeper);
    ros::spinOnce();
  }
  SpinRos(500, 10);

  ASSERT_TRUE(CollisionsEq(msg1, "map", 2));
  EXPECT_TRUE(CollisionEq(msg1.collisions[0], "robot1", "base_link_1",
                          "layer_1", "layer_1", 0, {}));
  EXPECT_TRUE(CollisionEq(msg1.collisions[1], "robot1", "base_link_2",
                          "layer_1", "layer_1", 1, {-1, 0}));
  ASSERT_TRUE(CollisionsEq(msg2, "world", 1));
  EXPECT_TRUE(CollisionEq(msg2.collisions[0], "robot1", "base_link_2",
                          "layer_1", "layer_1", 1, {-1, 0}));
  // w->DebugVisualize();
  // DebugVisualization::Get().Publish();
  // ros::spin();
}

/**
 * Test with a invalid body specified in the exclude list
 */
TEST_F(BumperPluginTest, invalid_A) {
  world_yaml = this_file_dir / fs::path("bumper_tests/invalid_A/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException& e) {
    std::cmatch match;
    std::string regex_str = ".*Body with name \"random_body\" does not exist.*";
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
  ros::init(argc, argv, "bumper_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
