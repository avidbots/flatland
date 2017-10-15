/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  service_manager_test.cpp
 * @brief Testing service manager functionalities
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

#include <flatland_msgs/DeleteModel.h>
#include <flatland_msgs/MoveModel.h>
#include <flatland_msgs/SpawnModel.h>
#include <flatland_server/simulation_manager.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <regex>
#include <thread>

namespace fs = boost::filesystem;
using namespace flatland_server;

class ServiceManagerTest : public ::testing::Test {
 protected:
  SimulationManager* sim_man;
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  boost::filesystem::path robot_yaml;
  Timekeeper timekeeper;
  ros::NodeHandle nh;
  ros::ServiceClient client;
  std::thread simulation_thread;

  void SetUp() override {
    sim_man = nullptr;
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    timekeeper.SetMaxStepSize(1.0);
  }

  void TearDown() override {
    StopSimulationThread();
    if (sim_man) delete sim_man;
  }

  void StartSimulationThread() {
    sim_man =
        new SimulationManager(world_yaml.string(), 1000, 1 / 1000.0, false, 0);
    simulation_thread = std::thread(&ServiceManagerTest::SimulationThread,
                                    dynamic_cast<ServiceManagerTest*>(this));
  }

  void StopSimulationThread() {
    sim_man->Shutdown();
    simulation_thread.join();
  }

  void SimulationThread() { sim_man->Main(); }
};

/**
 * Testing service for loading a model which should succeed
 */
TEST_F(ServiceManagerTest, spawn_valid_model) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  robot_yaml = this_file_dir /
               fs::path("load_world_tests/simple_test_A/person.model.yaml");

  flatland_msgs::SpawnModel srv;

  srv.request.name = "service_manager_test_robot";
  srv.request.ns = "robot123";
  srv.request.yaml_path = robot_yaml.string();
  srv.request.pose.x = 101.1;
  srv.request.pose.y = 102.1;
  srv.request.pose.theta = 0.23;

  client = nh.serviceClient<flatland_msgs::SpawnModel>("spawn_model");

  // Threading is required since client.call blocks executing until return
  StartSimulationThread();

  ros::service::waitForService("spawn_model", 1000);
  ASSERT_TRUE(client.call(srv));

  ASSERT_TRUE(srv.response.success);
  ASSERT_STREQ("", srv.response.message.c_str());

  World* w = sim_man->world_;
  ASSERT_EQ(5, w->models_.size());
  EXPECT_STREQ("service_manager_test_robot", w->models_[4]->name_.c_str());
  EXPECT_STREQ("robot123", w->models_[4]->namespace_.c_str());
  EXPECT_FLOAT_EQ(101.1,
                  w->models_[4]->bodies_[0]->physics_body_->GetPosition().x);
  EXPECT_FLOAT_EQ(102.1,
                  w->models_[4]->bodies_[0]->physics_body_->GetPosition().y);
  EXPECT_FLOAT_EQ(0.23, w->models_[4]->bodies_[0]->physics_body_->GetAngle());
  EXPECT_EQ(1, w->models_[4]->bodies_.size());
}

/**
 * Testing service for loading a model which should fail
 */
TEST_F(ServiceManagerTest, spawn_invalid_model) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  robot_yaml = this_file_dir / fs::path("random_path/turtlebot.model.yaml");

  flatland_msgs::SpawnModel srv;

  srv.request.name = "service_manager_test_robot";
  srv.request.yaml_path = robot_yaml.string();
  srv.request.pose.x = 1;
  srv.request.pose.y = 2;
  srv.request.pose.theta = 3;

  client = nh.serviceClient<flatland_msgs::SpawnModel>("spawn_model");

  StartSimulationThread();

  ros::service::waitForService("spawn_model", 1000);
  ASSERT_TRUE(client.call(srv));

  ASSERT_FALSE(srv.response.success);

  std::cmatch match;
  std::string regex_str =
      "Flatland YAML: File does not exist, "
      "path=\".*/random_path/turtlebot.model.yaml\".*";
  std::regex regex(regex_str);
  EXPECT_TRUE(std::regex_match(srv.response.message.c_str(), match, regex))
      << "Error Message '" + srv.response.message + "'" +
             " did not match against regex '" + regex_str + "'";
}

/**
 * Testing service for moving a valid model
 */
TEST_F(ServiceManagerTest, move_model) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  flatland_msgs::MoveModel srv;
  srv.request.name = "turtlebot1";
  srv.request.pose.x = 5.5;
  srv.request.pose.y = 9.9;
  srv.request.pose.theta = 0.77;

  client = nh.serviceClient<flatland_msgs::MoveModel>("move_model");

  StartSimulationThread();

  ros::service::waitForService("move_model", 1000);
  ASSERT_TRUE(client.call(srv));

  ASSERT_TRUE(srv.response.success);

  World* w = sim_man->world_;
  EXPECT_NEAR(5.5, w->models_[0]->bodies_[0]->physics_body_->GetPosition().x,
              1e-2);
  EXPECT_NEAR(9.9, w->models_[0]->bodies_[0]->physics_body_->GetPosition().y,
              1e-2);
  EXPECT_NEAR(0.77, w->models_[0]->bodies_[0]->physics_body_->GetAngle(), 1e-2);
}

/**
 * Testing service for moving a nonexistent model
 */
TEST_F(ServiceManagerTest, move_nonexistent_model) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  flatland_msgs::MoveModel srv;
  srv.request.name = "not_a_robot";
  srv.request.pose.x = 4;
  srv.request.pose.y = 5;
  srv.request.pose.theta = 0;

  client = nh.serviceClient<flatland_msgs::MoveModel>("move_model");

  StartSimulationThread();

  ros::service::waitForService("move_model", 1000);
  ASSERT_TRUE(client.call(srv));

  ASSERT_FALSE(srv.response.success);
  EXPECT_STREQ(
      "Flatland World: failed to move model, model with name "
      "\"not_a_robot\" does not exist",
      srv.response.message.c_str());
}

/**
 * Testing service for deleting a model
 */
TEST_F(ServiceManagerTest, delete_model) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/load_dummy_test/world.yaml");

  flatland_msgs::DeleteModel srv;
  srv.request.name = "turtlebot1";

  client = nh.serviceClient<flatland_msgs::DeleteModel>("delete_model");

  StartSimulationThread();

  ros::service::waitForService("delete_model", 1000);
  ASSERT_TRUE(client.call(srv));

  ASSERT_TRUE(srv.response.success);
  World* w = sim_man->world_;
  // after deleting a mode, there should be one less model, and one less plugin
  ASSERT_EQ(w->models_.size(), 0);
  ASSERT_EQ(w->plugin_manager_.model_plugins_.size(), 0);
  int count = std::count_if(w->models_.begin(), w->models_.end(),
                            [](Model* m) { return m->name_ == "turtlebot1"; });
  ASSERT_EQ(count, 0);
}

/**
 * Testing service for deleting a model that does not exist, should fail
 */
TEST_F(ServiceManagerTest, delete_nonexistent_model) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/load_dummy_test/world.yaml");

  flatland_msgs::DeleteModel srv;
  srv.request.name = "random_model";

  client = nh.serviceClient<flatland_msgs::DeleteModel>("delete_model");

  StartSimulationThread();

  ros::service::waitForService("delete_model", 1000);
  ASSERT_TRUE(client.call(srv));

  ASSERT_FALSE(srv.response.success);
  EXPECT_STREQ(
      "Flatland World: failed to delete model, model with name "
      "\"random_model\" does not exist",
      srv.response.message.c_str());
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "service_manager_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
