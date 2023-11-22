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

#include <flatland_server/simulation_manager.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>

#include <chrono>
#include <flatland_msgs/srv/delete_model.hpp>
#include <flatland_msgs/srv/move_model.hpp>
#include <flatland_msgs/srv/spawn_model.hpp>
#include <regex>
#include <thread>

namespace fs = boost::filesystem;
using namespace flatland_server;
using namespace std::chrono_literals;

class ServiceManagerTest : public ::testing::Test
{
public:
  explicit ServiceManagerTest(const std::shared_ptr<rclcpp::Node> & node)
  : timekeeper(node), node(node)
  {
  }

  ServiceManagerTest() : ServiceManagerTest(rclcpp::Node::make_shared("test_service_manager")) {}

protected:
  SimulationManager * sim_man;
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  boost::filesystem::path robot_yaml;
  Timekeeper timekeeper;
  rclcpp::Node::SharedPtr node;
  std::thread simulation_thread;

  void SetUp() override
  {
    sim_man = nullptr;
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    timekeeper.SetMaxStepSize(1.0);
  }

  void TearDown() override
  {
    StopSimulationThread();
    delete sim_man;
  }

  void StartSimulationThread()
  {
    sim_man = new SimulationManager(node, world_yaml.string(), 1000, 1 / 1000.0, false, 0);
    simulation_thread =
      std::thread(&ServiceManagerTest::SimulationThread, dynamic_cast<ServiceManagerTest *>(this));
  }

  void StopSimulationThread()
  {
    sim_man->Shutdown();
    simulation_thread.join();
  }

  void SimulationThread() { sim_man->Main(); }
};

/**
 * Testing service for loading a model which should succeed
 */
TEST_F(ServiceManagerTest, spawn_valid_model)
{
  world_yaml = this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  robot_yaml = this_file_dir / fs::path("load_world_tests/simple_test_A/person.model.yaml");

  auto request = std::make_shared<flatland_msgs::srv::SpawnModel::Request>();
  request->name = "service_manager_test_robot";
  request->ns = "robot123";
  request->yaml_path = robot_yaml.string();
  request->pose.x = 101.1;
  request->pose.y = 102.1;
  request->pose.theta = 0.23;

  auto client = node->create_client<flatland_msgs::srv::SpawnModel>("spawn_model");

  // Threading is required since client.call blocks executing until return
  StartSimulationThread();

  ASSERT_TRUE(client->wait_for_service(1s));

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }
  auto response = result.get();
  ASSERT_TRUE(response->success);
  ASSERT_STREQ("", response->message.c_str());

  World * w = sim_man->world_;
  ASSERT_EQ(5UL, w->models_.size());
  EXPECT_STREQ("service_manager_test_robot", w->models_[4]->name_.c_str());
  EXPECT_STREQ("robot123", w->models_[4]->namespace_.c_str());
  EXPECT_FLOAT_EQ(101.1, w->models_[4]->bodies_[0]->physics_body_->GetPosition().x);
  EXPECT_FLOAT_EQ(102.1, w->models_[4]->bodies_[0]->physics_body_->GetPosition().y);
  EXPECT_FLOAT_EQ(0.23, w->models_[4]->bodies_[0]->physics_body_->GetAngle());
  EXPECT_EQ(1UL, w->models_[4]->bodies_.size());
}

/**
 * Testing service for loading a model which should fail
 */
TEST_F(ServiceManagerTest, spawn_invalid_model)
{
  world_yaml = this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  robot_yaml = this_file_dir / fs::path("random_path/turtlebot.model.yaml");

  auto request = std::make_shared<flatland_msgs::srv::SpawnModel::Request>();
  request->name = "service_manager_test_robot";
  request->yaml_path = robot_yaml.string();
  request->pose.x = 1;
  request->pose.y = 2;
  request->pose.theta = 3;

  auto client = node->create_client<flatland_msgs::srv::SpawnModel>("spawn_model");

  StartSimulationThread();

  ASSERT_TRUE(client->wait_for_service(1s));

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }
  auto response = result.get();
  ASSERT_FALSE(response->success);

  std::cmatch match;
  std::string regex_str =
    "Flatland YAML: File does not exist, "
    "path=\".*/random_path/turtlebot.model.yaml\".*";
  std::regex regex(regex_str);
  EXPECT_TRUE(std::regex_match(response->message.c_str(), match, regex))
    << "Error Message '" + response->message + "'" + " did not match against regex '" + regex_str +
         "'";
}

/**
 * Testing service for moving a valid model
 */
TEST_F(ServiceManagerTest, move_model)
{
  world_yaml = this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  auto request = std::make_shared<flatland_msgs::srv::MoveModel::Request>();
  request->name = "turtlebot1";
  request->pose.x = 5.5;
  request->pose.y = 9.9;
  request->pose.theta = 0.77;

  auto client = node->create_client<flatland_msgs::srv::MoveModel>("move_model");

  StartSimulationThread();

  ASSERT_TRUE(client->wait_for_service(1s));

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }
  auto response = result.get();
  ASSERT_TRUE(response->success);

  World * w = sim_man->world_;
  EXPECT_NEAR(5.5, w->models_[0]->bodies_[0]->physics_body_->GetPosition().x, 1e-2);
  EXPECT_NEAR(9.9, w->models_[0]->bodies_[0]->physics_body_->GetPosition().y, 1e-2);
  EXPECT_NEAR(0.77, w->models_[0]->bodies_[0]->physics_body_->GetAngle(), 1e-2);
}

/**
 * Testing service for moving a nonexistent model
 */
TEST_F(ServiceManagerTest, move_nonexistent_model)
{
  world_yaml = this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");

  auto request = std::make_shared<flatland_msgs::srv::MoveModel::Request>();
  request->name = "not_a_robot";
  request->pose.x = 4;
  request->pose.y = 5;
  request->pose.theta = 0;

  auto client = node->create_client<flatland_msgs::srv::MoveModel>("move_model");

  StartSimulationThread();

  ASSERT_TRUE(client->wait_for_service(1s));

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }
  auto response = result.get();
  ASSERT_FALSE(response->success);
  EXPECT_STREQ(
    "Flatland World: failed to move model, model with name "
    "\"not_a_robot\" does not exist",
    response->message.c_str());
}

/**
 * Testing service for deleting a model
 */
TEST_F(ServiceManagerTest, delete_model)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/load_dummy_test/world.yaml");

  auto request = std::make_shared<flatland_msgs::srv::DeleteModel::Request>();
  request->name = "turtlebot1";

  auto client = node->create_client<flatland_msgs::srv::DeleteModel>("delete_model");

  StartSimulationThread();

  ASSERT_TRUE(client->wait_for_service(1s));

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }
  auto response = result.get();
  ASSERT_TRUE(response->success);
  World * w = sim_man->world_;
  // after deleting a mode, there should be one less model, and one less plugin
  ASSERT_EQ(w->models_.size(), 0UL);
  ASSERT_EQ(w->plugin_manager_.model_plugins_.size(), 0UL);
  size_t count = std::count_if(
    w->models_.begin(), w->models_.end(), [](Model * m) { return m->name_ == "turtlebot1"; });
  ASSERT_EQ(count, 0UL);
}

/**
 * Testing service for deleting a model that does not exist, should fail
 */
TEST_F(ServiceManagerTest, delete_nonexistent_model)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/load_dummy_test/world.yaml");

  auto request = std::make_shared<flatland_msgs::srv::DeleteModel::Request>();
  request->name = "random_model";

  auto client = node->create_client<flatland_msgs::srv::DeleteModel>("delete_model");

  StartSimulationThread();

  ASSERT_TRUE(client->wait_for_service(1s));

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    FAIL();
  }
  auto response = result.get();
  ASSERT_FALSE(response->success);
  EXPECT_STREQ(
    "Flatland World: failed to delete model, model with name "
    "\"random_model\" does not exist",
    response->message.c_str());
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
