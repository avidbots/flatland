/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  tricycle_drive_test.cpp
 * @brief test tricycle drive plugin
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

#include <flatland_plugins/tricycle_drive.h>
#include <flatland_server/model_plugin.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace flatland_plugins;

class TricycleDrivePluginTest : public ::testing::Test {
 public:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  nav_msgs::Odometry odom;
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

  void GroundTruthSubscriberCB(const nav_msgs::Odometry& msg) { odom = msg; }

  void SpinRos(float hz, int iterations) {
    ros::WallRate rate(hz);
    for (unsigned int i = 0; i < iterations; i++) {
      ros::spinOnce();
      rate.sleep();
    }
  }
};

TEST_F(TricycleDrivePluginTest, load_test) {
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
      "flatland_server", "flatland_server::ModelPlugin");

  try {
    boost::shared_ptr<flatland_server::ModelPlugin> plugin =
        loader.createInstance("flatland_plugins::TricycleDrive");
  } catch (pluginlib::PluginlibException& e) {
    FAIL() << "Failed to load Tricycle Drive plugin. " << e.what();
  }
}

TEST_F(TricycleDrivePluginTest, drive_test_simple) {
  world_yaml =
      this_file_dir / fs::path("tricycle_drive_tests/world.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_1;
  TricycleDrivePluginTest* obj = dynamic_cast<TricycleDrivePluginTest*>(this);
  sub_1 = nh.subscribe("odometry/ground_truth", 1, &TricycleDrivePluginTest::GroundTruthSubscriberCB, obj);

  // This unit test's world+model file specifically loads the tricycle drive plugin first to make this easy
  TricycleDrive* td = dynamic_cast<TricycleDrive*>(w->plugin_manager_.model_plugins_[0].get());


  // Give ros a moment to set up subscribers/publishers
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  // Check odom is 0 linear, 0 angular
  EXPECT_NEAR(0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.0, odom.pose.pose.position.x, 0.01);

  // directly set the Twist message for velocity input
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = 0;  // rad/s


  // drive forward for 1 second
  cmd_vel.linear.x = 0.5;  // m/s
  td->TwistCallback(cmd_vel);
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  EXPECT_NEAR(0.5, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.5, odom.pose.pose.position.x, 0.01);  // should have driven 0.5m from the start


  // Stop
  cmd_vel.linear.x = 0.0;  // m/s
  td->TwistCallback(cmd_vel);
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  EXPECT_NEAR(0.0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.5, odom.pose.pose.position.x, 0.01);  // should have driven 0.5m from the start

}


TEST_F(TricycleDrivePluginTest, drive_test_vel_limit) {
  world_yaml =
      this_file_dir / fs::path("tricycle_drive_tests/world2.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_1;
  TricycleDrivePluginTest* obj = dynamic_cast<TricycleDrivePluginTest*>(this);
  sub_1 = nh.subscribe("odometry/ground_truth", 1, &TricycleDrivePluginTest::GroundTruthSubscriberCB, obj);

  // This unit test's world+model file specifically loads the tricycle drive plugin first to make this easy
  TricycleDrive* td = dynamic_cast<TricycleDrive*>(w->plugin_manager_.model_plugins_[0].get());


  // Give ros a moment to set up subscribers/publishers
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  // Check odom is 0 linear, 0 angular
  EXPECT_NEAR(0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.0, odom.pose.pose.position.x, 0.01);  // start position is 12m on x

  // directly set the Twist message for velocity input
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = 0;  // rad/s


  // drive forward for 1 second
  cmd_vel.linear.x = 0.5;  // m/s, but the limit is 0.2m/s
  td->TwistCallback(cmd_vel);
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  EXPECT_NEAR(0.2, odom.twist.twist.linear.x, 0.01);  // verify that we're being capped at 0.2m/s
  EXPECT_NEAR(0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.2, odom.pose.pose.position.x, 0.01);  // should have driven 0.2m from the start


  // Stop
  cmd_vel.linear.x = 0.0;  // m/s
  td->TwistCallback(cmd_vel);
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  EXPECT_NEAR(0.0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.2, odom.pose.pose.position.x, 0.01);  // should have driven 0.2m from the start

}


TEST_F(TricycleDrivePluginTest, drive_test_angular_limit) {
  world_yaml =
      this_file_dir / fs::path("tricycle_drive_tests/world3.yaml");

  Timekeeper timekeeper;
  timekeeper.SetMaxStepSize(0.01);
  w = World::MakeWorld(world_yaml.string());

  ros::NodeHandle nh;
  ros::Subscriber sub_1;
  TricycleDrivePluginTest* obj = dynamic_cast<TricycleDrivePluginTest*>(this);
  sub_1 = nh.subscribe("odometry/ground_truth", 1, &TricycleDrivePluginTest::GroundTruthSubscriberCB, obj);

  // This unit test's world+model file specifically loads the tricycle drive plugin first to make this easy
  TricycleDrive* td = dynamic_cast<TricycleDrive*>(w->plugin_manager_.model_plugins_[0].get());


  // Give ros a moment to set up subscribers/publishers
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  // Check odom is 0 linear, 0 angular
  EXPECT_NEAR(0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.0, odom.pose.pose.position.x, 0.01);

  // directly set the Twist message for velocity input
  geometry_msgs::Twist cmd_vel;

  // rotate front wheel for 1 seconds
  cmd_vel.angular.z = 0.5;  // rad/s, but the limit is 0.2rad/s
  td->TwistCallback(cmd_vel);
  for (unsigned int i = 0; i < 100; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }
  // front wheel should have rotated to 0.2 radians, but we didn't move
  EXPECT_NEAR(0.0, odom.twist.twist.linear.x, 0.01);
  EXPECT_NEAR(0.0, odom.twist.twist.angular.z, 0.01);
  EXPECT_NEAR(12.0, odom.pose.pose.position.x, 0.01);  // should not have moved


  // drive, rotating based on front wheel angle for 0.1 seconds
  cmd_vel.angular.z = 0.0;  // rad/s
  cmd_vel.linear.x = 0.5;  // m/s
  td->TwistCallback(cmd_vel);
  for (unsigned int i = 0; i < 10; i++) {
    w->Update(timekeeper);
    ros::spinOnce();
  }

  EXPECT_NEAR(0.5, odom.twist.twist.linear.x, 0.01);  //driving forward
  EXPECT_NEAR(0.11, odom.twist.twist.angular.z, 0.01);   // robot body rotation due to wheel angle
  EXPECT_NEAR(12+cos(0.2)*0.5*0.1, odom.pose.pose.position.x, 0.01);  // drive
  EXPECT_NEAR(0+sin(0.2)*0.5*0.1, odom.pose.pose.position.y, 0.01);  // drive
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "tricycle_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
