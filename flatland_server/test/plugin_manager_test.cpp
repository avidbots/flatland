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
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;

class TestModelPlugin : public ModelPlugin {
 public:
  // variables used for testing
  double param_timestep_before;
  double param_timestep_after;
  Layer *param_layer;
  Model *param_model;
  b2Fixture *param_fixture_A;
  b2Fixture *param_fixture_B;

  std::map<std::string, bool> function_called;

  TestModelPlugin() { ClearTestingVariables(); }

  void ClearTestingVariables() {
    param_layer = nullptr;
    param_model = nullptr;
    param_fixture_A = nullptr;
    param_fixture_B = nullptr;

    function_called["OnInitialize"] = false;
    function_called["BeforePhysicsStep"] = false;
    function_called["AfterPhysicsStep"] = false;
    function_called["BeginContactWithMap"] = false;
    function_called["BeginContactWithModel"] = false;
    function_called["EndContactWithMap"] = false;
    function_called["EndContactWithModel"] = false;
    function_called["BeginContact"] = false;
    function_called["EndContact"] = false;
  }

  void OnInitialize(const YAML::Node &config) override {
    function_called["OnInitialize"] = true;
  }

  void BeforePhysicsStep(const Timekeeper &timekeeper) override {
    function_called["BeforePhysicsStep"] = true;
  }

  void AfterPhysicsStep(const Timekeeper &timekeeper) override {
    function_called["AfterPhysicsStep"] = true;
  }

  void BeginContactWithMap(Layer *layer, b2Fixture *layer_fixture,
                           b2Fixture *this_fixture,
                           b2Contact *contact) override {
    function_called["BeginContactWithMap"] = true;
    param_layer = layer;
    param_fixture_A = layer_fixture;
    param_fixture_B = this_fixture;
  }

  void BeginContactWithModel(Model *model, b2Fixture *model_fixture,
                             b2Fixture *this_fixture,
                             b2Contact *contact) override {
    function_called["BeginContactWithModel"] = true;
    param_model = model;
    param_fixture_A = model_fixture;
    param_fixture_B = this_fixture;
  }

  void EndContactWithMap(Layer *layer, b2Fixture *layer_fixture,
                         b2Fixture *this_fixture, b2Contact *contact) override {
    function_called["EndContactWithMap"] = true;
    param_layer = layer;
    param_fixture_A = layer_fixture;
    param_fixture_B = this_fixture;
  }

  void EndContactWithModel(Model *model, b2Fixture *model_fixture,
                           b2Fixture *this_fixture,
                           b2Contact *contact) override {
    function_called["EndContactWithModel"] = true;
    param_model = model;
    param_fixture_A = model_fixture;
    param_fixture_B = this_fixture;
  }

  void BeginContact(b2Contact *contact) override {
    function_called["BeginContact"] = true;
  }

  void EndContact(b2Contact *contact) override {
    function_called["EndContact"] = true;
  }
};

class PluginManagerTest : public ::testing::Test {
 protected:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  Timekeeper timekeeper;

  PluginManagerTest() {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }

  bool fltcmp(double n1, double n2) {
    bool ret = fabs(n1 - n2) < 1e-7;
    return ret;
  }

  // checks if tow maps have the same keys
  bool key_compare(std::map<std::string, bool> const &lhs,
                   std::map<std::string, bool> const &rhs) {
    auto pred = [](decltype(*lhs.begin()) a, decltype(a) b) {
      return a.first == b.first;
    };

    return lhs.size() == rhs.size() &&
           std::equal(lhs.begin(), lhs.end(), rhs.begin(), pred);
  }

  // Check the true/false if the function is called
  bool FunctionCallEq(TestModelPlugin *p,
                      std::map<std::string, bool> function_called) {
    if (!key_compare(p->function_called, function_called)) {
      printf("Two maps does not have the same keys (set of function)\n");
      return false;
    }

    for (const auto &func : p->function_called) {
      // printf("%s, Actual:%d Expected:%d\n", func.first.c_str(), func.second,
      //        function_called[func.first]);
      if (func.second != function_called[func.first]) {
        printf("%s is %s, expected to be %s\n", func.first.c_str(),
               func.second ? "called" : "not called",
               function_called[func.first] ? "called" : "not called");
        return false;
      }
    }
    return true;
  }

  // Checks the expected entities are collided
  bool FunctionParamEq(TestModelPlugin *p, Layer *layer, Model *model) {
    if (layer != p->param_layer) {
      printf("layer Actual:%p(%s) != Expected:%p(%s)\n", p->param_layer,
             p->param_layer ? p->param_layer->name_.c_str() : "null", layer,
             layer ? layer->name_.c_str() : "null");
      return false;
    }

    if (model != p->param_model) {
      printf("model Actual:%p(%s) != Expected:%p(%s)\n", p->param_model,
             p->param_model ? p->param_model->name_.c_str() : "null", model,
             model ? model->name_.c_str() : "null");
      return false;
    }

    return true;
  }
};

TEST_F(PluginManagerTest, collision_test) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/collision_test/world.yaml");
  timekeeper.SetMaxStepSize(1.0);
  World *w = World::MakeWorld(world_yaml.string());
  Layer *l = w->layers_[0];
  Model *m0 = w->models_[0];
  Model *m1 = w->models_[1];
  Body *b0 = m0->bodies_[0];
  Body *b1 = m1->bodies_[0];
  PluginManager *pm = &w->plugin_manager_;
  boost::shared_ptr<TestModelPlugin> shared_p(new TestModelPlugin());
  shared_p->Initialize("TestModelPlugin", "test_model_plugin", m0,
                       YAML::Node());
  pm->model_plugins_.push_back(shared_p);
  TestModelPlugin *p = shared_p.get();

  w->Update(timekeeper);

  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", true},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContactWithMap", true},
                                 {"BeginContactWithModel", false},
                                 {"EndContactWithMap", false},
                                 {"EndContactWithModel", false},
                                 {"BeginContact", true},
                                 {"EndContact", false}}));
  EXPECT_TRUE(FunctionParamEq(p, l, nullptr));
  EXPECT_EQ(p->param_fixture_B, b0->physics_body_->GetFixtureList());
  EXPECT_EQ(p->param_fixture_A->GetType(), b2Shape::e_edge);
  p->ClearTestingVariables();

  b0->physics_body_->SetLinearVelocity(b2Vec2(-1, 0));
  // takes two steps for Box2D to genreate collision events, not sure why
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", false},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContactWithMap", false},
                                 {"BeginContactWithModel", false},
                                 {"EndContactWithMap", true},
                                 {"EndContactWithModel", false},
                                 {"BeginContact", false},
                                 {"EndContact", true}}));
  EXPECT_TRUE(FunctionParamEq(p, l, nullptr));
  EXPECT_EQ(p->param_fixture_B, b0->physics_body_->GetFixtureList());
  EXPECT_EQ(p->param_fixture_A->GetType(), b2Shape::e_edge);
  p->ClearTestingVariables();

  b0->physics_body_->SetLinearVelocity(b2Vec2(0, 0));
  b1->physics_body_->SetLinearVelocity(b2Vec2(0, -0.5));
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", false},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContactWithMap", false},
                                 {"BeginContactWithModel", true},
                                 {"EndContactWithMap", false},
                                 {"EndContactWithModel", false},
                                 {"BeginContact", true},
                                 {"EndContact", false}}));
  EXPECT_TRUE(FunctionParamEq(p, nullptr, m1));
  EXPECT_EQ(p->param_fixture_A, b1->physics_body_->GetFixtureList());
  EXPECT_EQ(p->param_fixture_B, b0->physics_body_->GetFixtureList());
  p->ClearTestingVariables();

  b0->physics_body_->SetLinearVelocity(b2Vec2(0, 0));
  b1->physics_body_->SetLinearVelocity(b2Vec2(0, -1));
  w->Update(timekeeper);
  w->Update(timekeeper);
  EXPECT_TRUE(FunctionCallEq(p, {{"OnInitialize", false},
                                 {"BeforePhysicsStep", true},
                                 {"AfterPhysicsStep", true},
                                 {"BeginContactWithMap", false},
                                 {"BeginContactWithModel", false},
                                 {"EndContactWithMap", false},
                                 {"EndContactWithModel", true},
                                 {"BeginContact", false},
                                 {"EndContact", true}}));
  EXPECT_TRUE(FunctionParamEq(p, nullptr, m1));
  EXPECT_EQ(p->param_fixture_A, b1->physics_body_->GetFixtureList());
  EXPECT_EQ(p->param_fixture_B, b0->physics_body_->GetFixtureList());
  p->ClearTestingVariables();

  // w->DebugVisualize();
  // DebugVisualization::Get().Publish();
  // ros::spin();
}

TEST_F(PluginManagerTest, load_dummy_test) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/load_dummy_test/world.yaml");

  World *w = World::MakeWorld(world_yaml.string());

  ModelPlugin *p = w->plugin_manager_.model_plugins_[0].get();

  EXPECT_STREQ(p->type_.c_str(), "DummyModelPlugin");
  EXPECT_STREQ(p->name_.c_str(), "dummy_test_plugin");
}

TEST_F(PluginManagerTest, plugin_throws_exception) {
  world_yaml =
      this_file_dir /
      fs::path("plugin_manager_tests/plugin_throws_exception/world.yaml");

  try {
    World *w = World::MakeWorld(world_yaml.string());
    delete w;
    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException &e) {
    // do a regex match against error message
    std::string regex_str =
        ".*dummy_param_float must be dummy_test_123456, instead it was "
        "\"wrong_message\".*";
    std::cmatch match;
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
        << "Exception Message '" + std::string(e.what()) + "'" +
               " did not match against regex '" + regex_str + "'";
  } catch (const std::exception &e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, nonexistent_plugin) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/nonexistent_plugin/world.yaml");

  try {
    World *w = World::MakeWorld(world_yaml.string());
    delete w;
    FAIL() << "Expected an exception, but none were raised";
  } catch (const PluginException &e) {
    std::cmatch match;
    std::string regex_str =
        ".*RandomPlugin with base class type flatland_server::ModelPlugin does "
        "not exist.*";
    std::regex regex(regex_str);
    EXPECT_TRUE(std::regex_match(e.what(), match, regex))
        << "Exception Message '" + std::string(e.what()) + "'" +
               " did not match against regex '" + regex_str + "'";
  } catch (const std::exception &e) {
    ADD_FAILURE() << "Was expecting a PluginException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

TEST_F(PluginManagerTest, invalid_plugin_yaml) {
  world_yaml = this_file_dir /
               fs::path("plugin_manager_tests/invalid_plugin_yaml/world.yaml");

  try {
    World *w = World::MakeWorld(world_yaml.string());
    delete w;
    FAIL() << "Expected an exception, but none were raised";
  } catch (const YAMLException &e) {
    EXPECT_STREQ(e.what(), "Flatland YAML: Missing plugin name");
  } catch (const std::exception &e) {
    ADD_FAILURE() << "Was expecting a YAMLException, another exception was "
                     "caught instead: "
                  << e.what();
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ros::init(argc, argv, "plugin_manager_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
