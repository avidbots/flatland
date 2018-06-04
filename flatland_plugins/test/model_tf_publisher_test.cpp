/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  model_tf_publisher_test.cpp
 * @brief test model TF publisher plugin
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

#include <flatland_plugins/model_tf_publisher.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/TransformStamped.h>
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class ModelTfPublisherTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
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

  // Test if transform equals to expected
  bool TfEq(const geometry_msgs::TransformStamped& tf, float x, float y,
            float a) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(tf.transform.rotation, q);
    tf::Matrix3x3 rot_matrix(q);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);

    if (!fltcmp(x, tf.transform.translation.x) ||
        !fltcmp(y, tf.transform.translation.y) ||
        !fltcmp(0, tf.transform.translation.z) || !fltcmp(roll, 0) ||
        !fltcmp(pitch, 0) || !fltcmp(yaw, a)) {
      printf("Transformation\n");
      printf("Actual: x=%f y=%f z=%f, roll=%f pitch=%f yaw=%f\n",
             tf.transform.translation.x, tf.transform.translation.y,
             tf.transform.translation.z, roll, pitch, yaw);
      printf("Expected: x=%f y=%f z=%f, roll=%f pitch=%f yaw=%f\n", x, y, 0.0,
             0.0, 0.0, a);
      return false;
    }

    return true;
  }
};

/**
 * Test the transformation for the model robot in a given plugin configuration
 */
TEST_F(ModelTfPublisherTest, tf_publish_test_A) {
  world_yaml =
      this_file_dir /
      fs::path("model_tf_publisher_tests/tf_publish_test_A/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(1.0);
  w = World::MakeWorld(world_yaml.string());
  ModelTfPublisher* p = dynamic_cast<ModelTfPublisher*>(
      w->plugin_manager_.model_plugins_[0].get());

  EXPECT_DOUBLE_EQ(5000.0, p->update_rate_);
  EXPECT_STREQ("antenna", p->reference_body_->name_.c_str());

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped tf_world_to_base;
  geometry_msgs::TransformStamped tf_world_to_antenna;
  geometry_msgs::TransformStamped tf_base_to_left_wheel;
  geometry_msgs::TransformStamped tf_base_to_right_wheel;
  geometry_msgs::TransformStamped tf_base_to_front_bumper;
  geometry_msgs::TransformStamped tf_base_to_rear_bumper;

  // let it spin for 10 times to make sure the message gets through
  ros::WallRate rate(500);
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
    rate.sleep();
  }

  // check for the transformations that should exist
  tf_world_to_base =
      tf_buffer.lookupTransform("world", "my_robot_base", ros::Time(0));
  tf_world_to_antenna =
      tf_buffer.lookupTransform("world", "my_robot_antenna", ros::Time(0));
  tf_base_to_left_wheel = tf_buffer.lookupTransform(
      "my_robot_base", "my_robot_left_wheel", ros::Time(0));
  tf_base_to_right_wheel = tf_buffer.lookupTransform(
      "my_robot_base", "my_robot_right_wheel", ros::Time(0));

  // check for the transformations that should not exist
  try {
    tf_base_to_front_bumper = tf_buffer.lookupTransform(
        "my_robot_base", "my_robot_front_bumper", ros::Time(0));
    ADD_FAILURE() << "Expected an exception, but none were raised";
  } catch (const tf2::TransformException& e) {
    EXPECT_STREQ(
        "\"my_robot_front_bumper\" passed to lookupTransform argument "
        "source_frame does not exist. ",
        e.what());
  }

  try {
    tf_base_to_rear_bumper = tf_buffer.lookupTransform(
        "my_robot_base", "my_robot_rear_bumper", ros::Time(0));
    ADD_FAILURE() << "Expected an exception, but none were raised";
  } catch (const tf2::TransformException& e) {
    EXPECT_STREQ(
        "\"my_robot_rear_bumper\" passed to lookupTransform argument "
        "source_frame does not exist. ",
        e.what());
  }

  // check transformations are correct
  EXPECT_TRUE(TfEq(tf_world_to_base, 8, 6, -0.575958653));
  EXPECT_TRUE(TfEq(tf_world_to_antenna, 8, 6, -0.575958653));
  EXPECT_TRUE(TfEq(tf_base_to_left_wheel, -0.25, 1, 0));
  EXPECT_TRUE(TfEq(tf_base_to_right_wheel, -0.25, -1, 0));
}

/**
 * Test the transformation for the model robot in another plugin configuration
 */
TEST_F(ModelTfPublisherTest, tf_publish_test_B) {
  world_yaml =
      this_file_dir /
      fs::path("model_tf_publisher_tests/tf_publish_test_B/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(1.0);
  w = World::MakeWorld(world_yaml.string());
  ModelTfPublisher* p = dynamic_cast<ModelTfPublisher*>(
      w->plugin_manager_.model_plugins_[0].get());

  EXPECT_DOUBLE_EQ(std::numeric_limits<double>::infinity(), p->update_rate_);
  EXPECT_STREQ("base", p->reference_body_->name_.c_str());

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped tf_map_to_base;
  geometry_msgs::TransformStamped tf_base_to_antenna;
  geometry_msgs::TransformStamped tf_base_to_left_wheel;
  geometry_msgs::TransformStamped tf_base_to_right_wheel;
  geometry_msgs::TransformStamped tf_base_to_front_bumper;
  geometry_msgs::TransformStamped tf_base_to_rear_bumper;

  // let it spin for 10 times to make sure the message gets through
  ros::WallRate rate(500);
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
    rate.sleep();
  }

  tf_base_to_antenna =
      tf_buffer.lookupTransform("base", "antenna", ros::Time(0));
  tf_base_to_left_wheel =
      tf_buffer.lookupTransform("base", "left_wheel", ros::Time(0));
  tf_base_to_right_wheel =
      tf_buffer.lookupTransform("base", "right_wheel", ros::Time(0));
  tf_base_to_front_bumper =
      tf_buffer.lookupTransform("base", "front_bumper", ros::Time(0));
  tf_base_to_rear_bumper =
      tf_buffer.lookupTransform("base", "rear_bumper", ros::Time(0));

  try {
    tf_map_to_base = tf_buffer.lookupTransform("map", "base", ros::Time(0));
    ADD_FAILURE() << "Expected an exception, but none were raised";
  } catch (const tf2::TransformException& e) {
    EXPECT_STREQ(
        "\"map\" passed to lookupTransform argument target_frame does not "
        "exist. ",
        e.what());
  }

  EXPECT_TRUE(TfEq(tf_base_to_antenna, 0, 0, 0));
  EXPECT_TRUE(TfEq(tf_base_to_left_wheel, -0.25, 1, 0));
  EXPECT_TRUE(TfEq(tf_base_to_right_wheel, -0.25, -1, 0));
  EXPECT_TRUE(TfEq(tf_base_to_front_bumper, 2, 0, 0));
  EXPECT_TRUE(TfEq(tf_base_to_rear_bumper, -2, 0, 0));
}

/**
 * Test the transformation for the provided model yaml, which will fail due
 * to a nonexistent reference body
 */
TEST_F(ModelTfPublisherTest, invalid_A) {
  world_yaml =
      this_file_dir / fs::path("model_tf_publisher_tests/invalid_A/world.yaml");

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

/**
 * Test the transformation for the provided model yaml, which will fail due
 * to a nonexistent body specified in the exclude list
 */
TEST_F(ModelTfPublisherTest, invalid_B) {
  world_yaml =
      this_file_dir / fs::path("model_tf_publisher_tests/invalid_B/world.yaml");

  try {
    w = World::MakeWorld(world_yaml.string());

    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException& e) {
    std::cmatch match;
    std::string regex_str =
        ".*Body with name \"random_body_1\" does not exist.*";
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
  ros::init(argc, argv, "model_tf_plugin_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
