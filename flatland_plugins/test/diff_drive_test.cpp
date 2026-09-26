/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  diff_drive_test.cpp
 * @brief test diff drive plugin
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

#include <flatland_plugins/diff_drive.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(DiffDrivePluginTest, load_test)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_diff_drive_plugin");
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
    "flatland_server", "flatland_server::ModelPlugin");

  try {
    std::shared_ptr<flatland_server::ModelPlugin> plugin =
      loader.createSharedInstance("flatland_plugins::DiffDrive");
  } catch (pluginlib::PluginlibException & e) {
    FAIL() << "Failed to load diff drive Drive plugin. " << e.what();
  }
}

class DiffDriveOdomTest : public ::testing::Test
{
protected:
  void CheckStationaryNoise(bool stationary_noise, double angular_command)
  {
    auto node = rclcpp::Node::make_shared("test_diff_drive_stationary_noise");
    auto world_path = std::filesystem::path(__FILE__).parent_path() / "update_timer_test/world.yaml";
    std::unique_ptr<flatland_server::World> world(
      flatland_server::World::MakeWorld(node, world_path.string()));
    world->models_[0]->GetBody("base")->physics_body_->SetTransform(flatland::b2Vec2(2.0f, 3.0f), 0.0f);

    YAML::Node config;
    config["body"] = "base";
    config["odom_include_pose"] = true;
    config["odom_pose_noise"] = std::vector<double>{0.04, 0.04, 0.04};
    config["odom_twist_noise"] = std::vector<double>{0.04, 0.04, 0.04};
    if (stationary_noise) {
      config["odom_stationary_noise"] = true;
    }

    auto plugin = std::make_shared<flatland_plugins::DiffDrive>();
    plugin->Initialize(node, "DiffDrive", "drive", world->models_[0], config);
    world->plugin_manager_.model_plugins_.push_back(plugin);
    EXPECT_EQ(plugin->odom_stationary_noise_, stationary_noise);
    plugin->twist_msg_->twist.angular.z = angular_command;

    nav_msgs::msg::Odometry::SharedPtr received_odom;
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr received_twist;
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      plugin->odom_pub_->get_topic_name(), 1,
      [&received_odom](nav_msgs::msg::Odometry::SharedPtr message) { received_odom = message; });
    auto twist_sub = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      plugin->twist_pub_->get_topic_name(), 1,
      [&received_twist](geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr message) {
        received_twist = message;
      });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    flatland_server::Timekeeper timekeeper(node);
    timekeeper.SetMaxStepSize(0.01);
    rclcpp::WallRate rate(100);
    for (unsigned int attempt = 0; attempt < 30 && (!received_odom || !received_twist); ++attempt) {
      world->Update(timekeeper);
      executor.spin_some();
      rate.sleep();
    }

    ASSERT_NE(received_odom, nullptr);
    ASSERT_NE(received_twist, nullptr);
    EXPECT_EQ(received_odom->twist.covariance[0], 0.04);
    EXPECT_EQ(received_twist->twist.covariance[0], 0.04);
    if (!stationary_noise && angular_command == 0.0) {
      EXPECT_EQ(received_odom->pose.pose.position.x, plugin->ground_truth_msg_.pose.pose.position.x);
      EXPECT_EQ(received_odom->pose.pose.position.y, plugin->ground_truth_msg_.pose.pose.position.y);
      EXPECT_EQ(received_odom->pose.pose.orientation.w, plugin->ground_truth_msg_.pose.pose.orientation.w);
      EXPECT_EQ(received_odom->twist.twist.linear.x, 0.0);
      EXPECT_EQ(received_odom->twist.twist.linear.y, 0.0);
      EXPECT_EQ(received_odom->twist.twist.angular.z, 0.0);
      EXPECT_EQ(received_twist->twist.twist.linear.x, 0.0);
      EXPECT_EQ(received_twist->twist.twist.angular.z, 0.0);
    } else {
      EXPECT_NE(received_odom->pose.pose.position.x, plugin->ground_truth_msg_.pose.pose.position.x);
      EXPECT_NE(received_twist->twist.twist.linear.x, 0.0);
    }
  }

  void CheckOdom(bool include_pose)
  {
    auto node = rclcpp::Node::make_shared(
      include_pose ? "test_diff_drive_with_pose" : "test_diff_drive_without_pose");
    auto world_path = std::filesystem::path(__FILE__).parent_path() / "update_timer_test/world.yaml";
    std::unique_ptr<flatland_server::World> world(
      flatland_server::World::MakeWorld(node, world_path.string()));
    world->models_[0]->GetBody("base")->physics_body_->SetTransform(flatland::b2Vec2(2.0f, 3.0f), 0.0f);

    YAML::Node config;
    config["body"] = "base";
    config["odom_pose_noise"] = std::vector<double>{0.01, 0.02, 0.03};
    config["odom_twist_noise"] = std::vector<double>{0.01, 0.02, 0.03};
    if (include_pose) {
      config["odom_include_pose"] = true;
    }

    auto plugin = std::make_shared<flatland_plugins::DiffDrive>();
    plugin->Initialize(node, "DiffDrive", "drive", world->models_[0], config);
    world->plugin_manager_.model_plugins_.push_back(plugin);
    EXPECT_EQ(plugin->odom_include_pose_, include_pose);
    plugin->twist_msg_->twist.linear.x = 0.25;

    nav_msgs::msg::Odometry::SharedPtr received;
    auto subscription = node->create_subscription<nav_msgs::msg::Odometry>(
      plugin->odom_pub_->get_topic_name(), 1,
      [&received](nav_msgs::msg::Odometry::SharedPtr message) { received = message; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    flatland_server::Timekeeper timekeeper(node);
    timekeeper.SetMaxStepSize(0.01);
    rclcpp::WallRate rate(100);
    for (unsigned int attempt = 0; attempt < 30 && !received; ++attempt) {
      world->Update(timekeeper);
      executor.spin_some();
      rate.sleep();
    }

    ASSERT_NE(received, nullptr);
    EXPECT_EQ(received->twist.twist.linear.x, plugin->odom_msg_.twist.twist.linear.x);
    EXPECT_NE(received->twist.twist.linear.x, 0.0);
    EXPECT_EQ(received->twist.covariance, plugin->odom_msg_.twist.covariance);
    EXPECT_EQ(received->twist.covariance[0], 0.01);
    EXPECT_EQ(plugin->odom_msg_.pose.covariance[0], 0.01);
    EXPECT_EQ(plugin->odom_msg_.pose.covariance[7], 0.02);
    EXPECT_EQ(plugin->odom_msg_.pose.covariance[35], 0.03);
    EXPECT_GT(plugin->odom_msg_.pose.pose.position.x, 1.0);
    if (include_pose) {
      EXPECT_EQ(received->pose.pose.position.x, plugin->odom_msg_.pose.pose.position.x);
      EXPECT_EQ(received->pose.pose.orientation.w, plugin->odom_msg_.pose.pose.orientation.w);
      EXPECT_EQ(received->pose.covariance, plugin->odom_msg_.pose.covariance);
    } else {
      EXPECT_EQ(received->pose.pose.position.x, 0.0);
      EXPECT_EQ(received->pose.pose.orientation.w, 1.0);
      EXPECT_EQ(received->pose.covariance, geometry_msgs::msg::PoseWithCovariance().covariance);
    }
  }
};

TEST_F(DiffDriveOdomTest, PoseDisabledByDefault)
{
  CheckOdom(false);
}

TEST_F(DiffDriveOdomTest, PoseIncludedWhenEnabled)
{
  CheckOdom(true);
}

TEST_F(DiffDriveOdomTest, NoNoiseWhileStationaryByDefault)
{
  CheckStationaryNoise(false, 0.0);
}

TEST_F(DiffDriveOdomTest, StationaryNoiseWhenEnabled)
{
  CheckStationaryNoise(true, 0.0);
}

TEST_F(DiffDriveOdomTest, TurningInPlaceStillHasNoise)
{
  CheckStationaryNoise(false, 0.25);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
