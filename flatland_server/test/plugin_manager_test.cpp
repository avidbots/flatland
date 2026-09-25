/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  plugin_manager_test.cpp
 * @brief Testing plugin manager functionalities
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

#include <flatland_server/debug_visualization.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <regex>

namespace fs = std::filesystem;
using namespace flatland_server;

class TestModelPlugin : public ModelPlugin
{
public:
  // variables used for testing
  double timestep_before;
  double timestep_after;
  Entity * entity;
  flatland::b2Fixture * fixture_A;
  flatland::b2Fixture * fixture_B;

  std::map<std::string, bool> function_called;

  TestModelPlugin() { ClearTestingVariables(); }

  void ClearTestingVariables()
  {
    entity = nullptr;
    fixture_A = nullptr;
    fixture_B = nullptr;

    function_called["OnInitialize"] = false;
    function_called["BeforePhysicsStep"] = false;
    function_called["AfterPhysicsStep"] = false;
    function_called["BeginContact"] = false;
    function_called["EndContact"] = false;
    function_called["PostSolve"] = false;
  }

  void OnInitialize(const YAML::Node &) override { function_called["OnInitialize"] = true; }

  void BeforePhysicsStep(const Timekeeper &) override
  {
    function_called["BeforePhysicsStep"] = true;
  }

  void AfterPhysicsStep(const Timekeeper &) override
  {
    function_called["AfterPhysicsStep"] = true;
  }

  void BeginContact(flatland::b2Contact * contact) override
  {
    function_called["BeginContact"] = true;
    FilterContact(contact, entity, fixture_A, fixture_B);
  }

  void EndContact(flatland::b2Contact * contact) override
  {
    function_called["EndContact"] = true;
    FilterContact(contact, entity, fixture_A, fixture_B);
  }

  void PostSolve(flatland::b2Contact * contact, const flatland::b2ContactImpulse *) override
  {
    function_called["PostSolve"] = true;
    FilterContact(contact, entity, fixture_A, fixture_B);
  }
};

class PluginManagerTest : public ::testing::Test
{
protected:
  fs::path this_file_dir;
  fs::path world_yaml;
  World * w;

  void SetUp() override
  {
    this_file_dir = fs::path(__FILE__).parent_path();
    w = nullptr;
  }

  void TearDown() override
  {
    if (w != nullptr) {
      delete w;
    }
  }

  PluginManagerTest() { this_file_dir = fs::path(__FILE__).parent_path(); }

  bool fltcmp(double n1, double n2)
  {
    bool ret = std::fabs(n1 - n2) < 1e-7;
    return ret;
  }

  // checks if tow maps have the same keys
  bool key_compare(std::map<std::string, bool> const & lhs, std::map<std::string, bool> const & rhs)
  {
    auto pred = [](decltype(*lhs.begin()) a, decltype(a) b) { return a.first == b.first; };

    return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
  }

  // Check the true/false if the function is called
  bool FunctionCallEq(TestModelPlugin * p, std::map<std::string, bool> function_called)
  {
    if (!key_compare(p->function_called, function_called)) {
      printf("Two maps does not have the same keys (set of function)\n");
      return false;
    }

    for (const auto & func : p->function_called) {
      if (func.second != function_called[func.first]) {
        printf(
          "%s is %s, expected to be %s\n", func.first.c_str(),
          func.second ? "called" : "not called",
          function_called[func.first] ? "called" : "not called");
        return false;
      }
    }
    return true;
  }
};

/**
 * This test moves bodies around and test if all the correct functions are
 * called with the expected inputs
 */
TEST_F(PluginManagerTest, collision_test)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/collision_test/world.yaml");
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("PluginManagerTest_node");
  Timekeeper timekeeper(node);
  timekeeper.SetMaxStepSize(1.0);

  w = World::MakeWorld(node, world_yaml.string());
  Layer * l = w->layers_[0];
  Model * m0 = w->models_[0];
  Model * m1 = w->models_[1];
  Body * b0 = m0->bodies_[0];
  Body * b1 = m1->bodies_[0];
  PluginManager * pm = &w->plugin_manager_;
  std::shared_ptr<TestModelPlugin> shared_p(new TestModelPlugin());
  shared_p->Initialize(node, "TestModelPlugin", "test_model_plugin", m0, YAML::Node());
  pm->model_plugins_.push_back(shared_p);
  TestModelPlugin * p = shared_p.get();

  // step the world with everything at zero velocity, this should make sure
  // the collision callbacks gets called
  w->Update(timekeeper);
  w->Update(timekeeper);

  // model 0 is placed right on top of a layer edge at the begining. Thus,
  // begin contact should trigger, as well as before and after physics step.
  // Note that pre and post solve are never called because the fixtures are
  // set as sensors
  EXPECT_TRUE(FunctionCallEq(
    p, {{"OnInitialize", true},
        {"BeforePhysicsStep", true},
        {"AfterPhysicsStep", true},
        {"BeginContact", true},
        {"EndContact", false},
        {"PostSolve", false}}));
  EXPECT_EQ(p->entity, l);
  EXPECT_EQ(p->fixture_A, b0->physics_body_->GetFixtureList());
  EXPECT_EQ(p->fixture_B->GetType(), flatland::b2Shape::e_edge);
  p->ClearTestingVariables();

  // move the body 2m to the left over two 1s timesteps, this should remove any
  // contacts between the body and the layer
  b0->physics_body_->SetLinearVelocity(flatland::b2Vec2(-1, 0));
  // takes two steps for Box2D to genreate collision events, not sure why
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(
    p, {{"OnInitialize", false},
        {"BeforePhysicsStep", true},
        {"AfterPhysicsStep", true},
        {"BeginContact", false},
        {"EndContact", true},
        {"PostSolve", false}}));
  EXPECT_EQ(p->entity, l);
  EXPECT_EQ(p->fixture_A, b0->physics_body_->GetFixtureList());
  EXPECT_EQ(p->fixture_B->GetType(), flatland::b2Shape::e_edge);
  p->ClearTestingVariables();

  // move the body 1m down over 2 timesteps, this should place model 0 in
  // contact with model 1
  b0->physics_body_->SetLinearVelocity(flatland::b2Vec2(0, 0));
  b1->physics_body_->SetLinearVelocity(flatland::b2Vec2(0, -0.5));
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(
    p, {{"OnInitialize", false},
        {"BeforePhysicsStep", true},
        {"AfterPhysicsStep", true},
        {"BeginContact", true},
        {"EndContact", false},
        {"PostSolve", false}}));
  EXPECT_EQ(p->entity, m1);
  EXPECT_EQ(p->fixture_B, b1->physics_body_->GetFixtureList());
  EXPECT_EQ(p->fixture_A, b0->physics_body_->GetFixtureList());
  p->ClearTestingVariables();

  // move the body 2m down over 2 timesteps, this should clear any contacts for
  // model 0
  b0->physics_body_->SetLinearVelocity(flatland::b2Vec2(0, 0));
  b1->physics_body_->SetLinearVelocity(flatland::b2Vec2(0, -1));
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(
    p, {{"OnInitialize", false},
        {"BeforePhysicsStep", true},
        {"AfterPhysicsStep", true},
        {"BeginContact", false},
        {"EndContact", true},
        {"PostSolve", false}}));
  EXPECT_EQ(p->entity, m1);
  EXPECT_EQ(p->fixture_B, b1->physics_body_->GetFixtureList());
  EXPECT_EQ(p->fixture_A, b0->physics_body_->GetFixtureList());
  p->ClearTestingVariables();

  // Now we set model 0 fixture as not a sensor, enabling post-solve reports
  // in the contact listener in subsequent tests
  b0->physics_body_->GetFixtureList()->SetSensor(false);

  // now teleport the body for model 0 to (0, 0) which is right on top of a
  // layer edge, set zero velocity and step, this will cause the body
  // to begin contact with the layer, but you can't be sure if end contact
  // will be called
  b0->physics_body_->SetLinearVelocity(flatland::b2Vec2(0, 0));
  b0->physics_body_->SetTransform(flatland::b2Vec2(0, 0), 0);
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(
    p, {{"OnInitialize", false},
        {"BeforePhysicsStep", true},
        {"AfterPhysicsStep", true},
        {"BeginContact", true},
        {"EndContact", false},
        {"PostSolve", true}}));
  EXPECT_EQ(p->entity, l);
  EXPECT_EQ(p->fixture_A, b0->physics_body_->GetFixtureList());
  EXPECT_EQ(p->fixture_B->GetType(), flatland::b2Shape::e_edge);
  p->ClearTestingVariables();

  // w->DebugVisualize();
  // DebugVisualization::Get(node_)->Publish();
  // ros::spin();
}

TEST(PhysicsAdapterTest, sensorChangeKeepsOtherFixtureContact)
{
  struct ContactRecorder : flatland::b2ContactListener
  {
    int begin_count = 0;
    std::vector<std::pair<flatland::b2Fixture *, flatland::b2Fixture *>> ended;

    void BeginContact(flatland::b2Contact *) override { ++begin_count; }
    void EndContact(flatland::b2Contact * contact) override
    {
      ended.emplace_back(contact->GetFixtureA(), contact->GetFixtureB());
    }
  } recorder;

  flatland::b2World physics_world({0.0f, 0.0f});
  physics_world.SetContactListener(&recorder);
  flatland::b2BodyDef static_definition;
  auto * ground = physics_world.CreateBody(&static_definition);
  flatland::b2BodyDef dynamic_definition;
  dynamic_definition.type = flatland::b2_dynamicBody;
  auto * body = physics_world.CreateBody(&dynamic_definition);

  flatland::b2CircleShape left;
  left.m_p = {-2.0f, 0.0f};
  left.m_radius = 0.5f;
  flatland::b2CircleShape right;
  right.m_p = {2.0f, 0.0f};
  right.m_radius = 0.5f;
  ground->CreateFixture(&left, 0.0f);
  ground->CreateFixture(&right, 0.0f);

  flatland::b2FixtureDef sensor_definition;
  sensor_definition.isSensor = true;
  sensor_definition.shape = &left;
  auto * changed = body->CreateFixture(&sensor_definition);
  sensor_definition.shape = &right;
  auto * unaffected = body->CreateFixture(&sensor_definition);

  physics_world.Step(1.0f / 60.0f, 4);
  ASSERT_EQ(recorder.begin_count, 2);

  changed->SetSensor(false);
  ASSERT_EQ(recorder.ended.size(), 1u);
  EXPECT_TRUE(recorder.ended[0].first == changed || recorder.ended[0].second == changed);
  EXPECT_NE(recorder.ended[0].first, unaffected);
  EXPECT_NE(recorder.ended[0].second, unaffected);
  physics_world.Step(1.0f / 60.0f, 4);
  EXPECT_EQ(recorder.ended.size(), 1u);
}

TEST(PhysicsAdapterTest, destroyedNativeShapeEndsContact)
{
  struct ContactRecorder : flatland::b2ContactListener
  {
    int begin_count = 0;
    int end_count = 0;

    void BeginContact(flatland::b2Contact *) override { ++begin_count; }
    void EndContact(flatland::b2Contact *) override { ++end_count; }
  } recorder;

  flatland::b2World physics_world({0.0f, 0.0f});
  physics_world.SetContactListener(&recorder);
  flatland::b2BodyDef static_definition;
  auto * ground = physics_world.CreateBody(&static_definition);
  flatland::b2BodyDef dynamic_definition;
  dynamic_definition.type = flatland::b2_dynamicBody;
  auto * body = physics_world.CreateBody(&dynamic_definition);
  flatland::b2CircleShape circle;
  circle.m_radius = 0.5f;
  ground->CreateFixture(&circle, 0.0f);
  flatland::b2FixtureDef sensor_definition;
  sensor_definition.shape = &circle;
  sensor_definition.isSensor = true;
  auto * sensor = body->CreateFixture(&sensor_definition);

  physics_world.Step(1.0f / 60.0f, 4);
  ASSERT_EQ(recorder.begin_count, 1);
  b2DestroyShape(sensor->id_, true);
  physics_world.Step(1.0f / 60.0f, 4);
  EXPECT_EQ(recorder.end_count, 1);
}

TEST(PhysicsAdapterTest, loopCreatesAllSegments)
{
  flatland::b2World physics_world({0.0f, 0.0f});
  flatland::b2BodyDef static_definition;
  auto * body = physics_world.CreateBody(&static_definition);
  flatland::b2Vec2 corners[] = {
    {-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
  flatland::b2ChainShape loop;
  loop.CreateLoop(corners, 4);

  auto * fixture = body->CreateFixture(&loop, 0.0f);
  ASSERT_NE(fixture, nullptr);
  EXPECT_EQ(body->owned_fixtures_.size(), 4u);
}

TEST_F(PluginManagerTest, load_dummy_test)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/load_dummy_test/world.yaml");

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_node");
  w = World::MakeWorld(node, world_yaml.string());

  ModelPlugin * p = w->plugin_manager_.model_plugins_[0].get();

  EXPECT_STREQ(p->GetType().c_str(), "DummyModelPlugin");
  EXPECT_STREQ(p->GetName().c_str(), "dummy_test_plugin");
}

TEST_F(PluginManagerTest, plugin_throws_exception)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/plugin_throws_exception/world.yaml");

  try {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_node");
    w = World::MakeWorld(node, world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException & e) {
    // do a regex match against error message
    std::string regex_str =
      ".*dummy_param_float must be dummy_test_123456, instead it was "
      "\"wrong_message\".*";
    std::cmatch match;
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
      << "Exception Message '" + std::string(e.what()) + "'" + " did not match against regex '" +
           regex_str + "'";
  } catch (const std::exception & e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, nonexistent_plugin)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/nonexistent_plugin/world.yaml");

  try {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_node");
    w = World::MakeWorld(node, world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException & e) {
    std::cmatch match;
    std::string regex_str =
      ".*RandomPlugin with base class type flatland_server::ModelPlugin does "
      "not exist.*";
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
      << "Exception Message '" + std::string(e.what()) + "'" + " did not match against regex '" +
           regex_str + "'";
  } catch (const std::exception & e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, invalid_plugin_yaml)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/invalid_plugin_yaml/world.yaml");

  try {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_node");
    w = World::MakeWorld(node, world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const YAMLException & e) {
    EXPECT_STREQ(
      "Flatland YAML: Entry \"name\" does not exist (in model \"turtlebot1\" "
      "\"plugins\" index=0)",
      e.what());
  } catch (const std::exception & e) {
    ADD_FAILURE() << "Was expecting a YAMLException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, duplicate_plugin)
{
  world_yaml = this_file_dir / fs::path("plugin_manager_tests/duplicate_plugin/world.yaml");

  try {
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_node");
    w = World::MakeWorld(node, world_yaml.string());
    FAIL() << "Expected an exception, but none were raised";
  } catch (const YAMLException & e) {
    EXPECT_STREQ(
      "Flatland YAML: Invalid \"plugins\" in \"turtlebot1\" model, plugin "
      "with name \"dummy_test_plugin\" already exists",
      e.what());
  } catch (const std::exception & e) {
    ADD_FAILURE() << "Was expecting a YAMLException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
