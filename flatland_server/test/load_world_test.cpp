/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  load_world_test.cpp
 * @brief Testing the load world functionality
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

#include <Box2D/Box2D.h>
#include <flatland_server/entity.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <regex>
#include <string>
#include <flatland_server/geometry.h>

namespace fs = boost::filesystem;
using namespace flatland_server;

class FlatlandServerLoadWorldTest : public ::testing::Test {
 protected:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;

  FlatlandServerLoadWorldTest() {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }

  // to test that the world instantiation will fail, and the exception
  // message matches the given regex string
  void test_yaml_fail(std::string regex_str) {
    try {
      World *w = World::make_world(world_yaml.string());

      delete w;

      FAIL() << "Expected YAMLException, it passed instead";
    } catch (YAMLException &e) {
      // do a regex match against error messages
      std::cmatch match;
      std::regex regex(regex_str);
      EXPECT_TRUE(std::regex_match(e.what(), match, regex))
          << "'" + std::string(e.what()) + "'" + " did not match against " +
                 "'" + regex_str + "'";
    } catch (...) {
      FAIL() << "Expected YAMLException, another exception was caught instead";
    }
  }

  bool float_cmp(double n1, double n2) {
    bool ret = fabs(n1 - n2) < 1e-3;
    return ret;
  }

  // return the id if found, -1 otherwise
  int does_edge_exist(const b2EdgeShape &edge,
                      const std::vector<std::pair<b2Vec2, b2Vec2>> &edges) {
    for (int i = 0; i < edges.size(); i++) {
      auto e = edges[i];
      if ((float_cmp(edge.m_vertex1.x, e.first.x) &&
           float_cmp(edge.m_vertex1.y, e.first.y) &&
           float_cmp(edge.m_vertex2.x, e.second.x) &&
           float_cmp(edge.m_vertex2.y, e.second.y)) ||

          (float_cmp(edge.m_vertex1.x, e.second.x) &&
           float_cmp(edge.m_vertex1.y, e.second.y) &&
           float_cmp(edge.m_vertex2.x, e.first.x) &&
           float_cmp(edge.m_vertex2.y, e.first.y))) {
        return i;
      }
    }

    return -1;
  }

  // checks if one list of edges completely matches the content
  // of the other list
  bool do_edges_exactly_match(
      const std::vector<b2EdgeShape> &edges1,
      const std::vector<std::pair<b2Vec2, b2Vec2>> &edges2) {
    std::vector<std::pair<b2Vec2, b2Vec2>> edges_cpy = edges2;
    for (int i = 0; i < edges1.size(); i++) {
      auto e = edges1[i];
      int ret_idx = does_edge_exist(e, edges_cpy);

      if (ret_idx < 0) {
        b2Vec2 v1_tf = e.m_vertex1;
        b2Vec2 v2_tf = e.m_vertex2;
        return false;
      }

      edges_cpy.erase(edges_cpy.begin() + ret_idx);
    }

    if (edges_cpy.size() == 0) {
      return true;
    } else {
      return false;
    }
  }

  // transform edge to 
  void transform_edge_to_global(b2Body *body, b2EdgeShape *edge) {
    RotateTranslate transform = Geometry::createTransform(
      body->GetPosition().x, body->GetPosition().y, body->GetAngle());

      edge->m_vertex1 = Geometry::transform(edge->m_vertex1, transform);
      edge->m_vertex2 = Geometry::transform(edge->m_vertex2, transform);
  }
};

/**
 * This test loads the world, layers, models (TODO) from the given world
 * yaml file and checks that all configurations, data, and calculations are
 * correct after instantiation
 */
TEST_F(FlatlandServerLoadWorldTest, simple_test_A) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");
  World *w = World::make_world(world_yaml.string());

  ASSERT_EQ(w->layers_.size(), 2);

  // check that layer 0 settings are loaded correctly
  EXPECT_STREQ(w->layers_[0]->name_.c_str(), "2d");
  EXPECT_EQ(w->layers_[0]->type(), Entity::Type::LAYER);
  EXPECT_DOUBLE_EQ(w->layers_[0]->body_->color_[0], 0);
  EXPECT_DOUBLE_EQ(w->layers_[0]->body_->color_[1], 1);
  EXPECT_DOUBLE_EQ(w->layers_[0]->body_->color_[2], 0);
  EXPECT_DOUBLE_EQ(w->layers_[0]->body_->color_[3], 0);
  EXPECT_FALSE(w->layers_[0]->bitmap_.empty());
  EXPECT_EQ(w->layers_[0]->bitmap_.rows, 5);
  EXPECT_EQ(w->layers_[0]->bitmap_.cols, 5);
  EXPECT_DOUBLE_EQ(w->layers_[0]->resolution_, 0.05);
  EXPECT_DOUBLE_EQ(w->layers_[0]->occupied_thresh_, 0.65);
  EXPECT_DOUBLE_EQ(w->layers_[0]->free_thresh_, 0.196);

  // check that layer 1 settings are loaded correctly
  EXPECT_STREQ(w->layers_[1]->name_.c_str(), "3d");
  EXPECT_EQ(w->layers_[1]->type(), Entity::Type::LAYER);
  EXPECT_DOUBLE_EQ(w->layers_[1]->body_->color_[0], 1.0);
  EXPECT_DOUBLE_EQ(w->layers_[1]->body_->color_[1], 0.0);
  EXPECT_DOUBLE_EQ(w->layers_[1]->body_->color_[2], 0.0);
  EXPECT_DOUBLE_EQ(w->layers_[1]->body_->color_[3], 0.5);
  EXPECT_FALSE(w->layers_[1]->bitmap_.empty());
  EXPECT_EQ(w->layers_[1]->bitmap_.rows, 5);
  EXPECT_EQ(w->layers_[1]->bitmap_.cols, 5);
  EXPECT_DOUBLE_EQ(w->layers_[1]->resolution_, 1.5);
  EXPECT_DOUBLE_EQ(w->layers_[1]->occupied_thresh_, 0.5153);
  EXPECT_DOUBLE_EQ(w->layers_[1]->free_thresh_, 0.2234);

  // check that bitmap is transformed correctly. This involves flipping the y
  // coordinates and apply the resolution. Note that the translation and
  // rotation is performed internally by Box2D
  std::vector<std::pair<b2Vec2, b2Vec2>> layer0_expected_edges = {
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.00, 0.25), b2Vec2(0.25, 0.25)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.05, 0.20), b2Vec2(0.20, 0.20)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.10, 0.15), b2Vec2(0.15, 0.15)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.10, 0.10), b2Vec2(0.15, 0.10)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.05, 0.05), b2Vec2(0.20, 0.05)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.00, 0.00), b2Vec2(0.25, 0.00)),

    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.00, 0.25), b2Vec2(0.00, 0.00)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.05, 0.20), b2Vec2(0.05, 0.05)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.10, 0.15), b2Vec2(0.10, 0.10)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.15, 0.15), b2Vec2(0.15, 0.10)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.20, 0.20), b2Vec2(0.20, 0.05)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0.25, 0.25), b2Vec2(0.25, 0.00))
  };

  std::vector<b2EdgeShape> layer0_edges;
  for (b2Fixture *f = w->layers_[0]->body_->physics_body_->GetFixtureList(); f;
       f = f->GetNext()) {
    b2EdgeShape e = *(dynamic_cast<b2EdgeShape *>(f->GetShape()));
    layer0_edges.push_back(e);

    // check that collision groups are correctly assigned
    EXPECT_EQ(f->GetFilterData().categoryBits, 0x1);
    EXPECT_EQ(f->GetFilterData().maskBits, 0x1);
  }
  EXPECT_EQ(layer0_edges.size(),
            layer0_expected_edges.size());
  EXPECT_TRUE(do_edges_exactly_match(layer0_edges,
                                     layer0_expected_edges));

  // layer[1] has origin of [0, 0, 0], so there should be no transform, just
  // apply the inversion of y coordinates and scale by resolution
  std::vector<std::pair<b2Vec2, b2Vec2>> layer1_expected_edges = {
      std::pair<b2Vec2, b2Vec2>(b2Vec2(0.0, 7.5), b2Vec2(1.5, 7.5)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(0.0, 7.5), b2Vec2(0.0, 4.5)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(0.0, 4.5), b2Vec2(4.5, 4.5)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(4.5, 4.5), b2Vec2(4.5, 1.5)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(6.0, 3.0), b2Vec2(6.0, 6.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(6.0, 6.0), b2Vec2(1.5, 6.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(1.5, 7.5), b2Vec2(1.5, 6.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(3.0, 3.0), b2Vec2(6.0, 3.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(3.0, 0.0), b2Vec2(3.0, 3.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(1.5, 1.5), b2Vec2(4.5, 1.5)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(1.5, 0.0), b2Vec2(3.0, 0.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(1.5, 1.5), b2Vec2(1.5, 0.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(6.0, 1.5), b2Vec2(7.5, 1.5)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(6.0, 1.5), b2Vec2(6.0, 0.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(7.5, 1.5), b2Vec2(7.5, 0.0)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(6.0, 0.0), b2Vec2(7.5, 0.0))};

  std::vector<b2EdgeShape> layer1_edges;
  for (b2Fixture *f = w->layers_[1]->body_->physics_body_->GetFixtureList(); f;
       f = f->GetNext()) {
    b2EdgeShape e = *(dynamic_cast<b2EdgeShape *>(f->GetShape()));
    layer1_edges.push_back(e);

    // check that collision groups are correctly assigned
    EXPECT_EQ(f->GetFilterData().categoryBits, 0x2);
    EXPECT_EQ(f->GetFilterData().maskBits, 0x2);
  }
  EXPECT_EQ(layer1_edges.size(),
            layer1_expected_edges.size());
  EXPECT_TRUE(do_edges_exactly_match(layer1_edges,
                                     layer1_expected_edges));

  delete w;
}

/**
 * This test tries to loads a non-existent world yaml file. It should throw
 * an exception
 */
TEST_F(FlatlandServerLoadWorldTest, wrong_world_path) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/random_path/world.yaml");
  test_yaml_fail("Error loading.*world.yaml.*bad file");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(FlatlandServerLoadWorldTest, world_invalid_A) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_A/world.yaml");
  test_yaml_fail("Missing/invalid world param \"properties\"");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(FlatlandServerLoadWorldTest, world_invalid_B) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_B/world.yaml");
  test_yaml_fail("Missing/invalid \"color\" in 2d layer");
}

/**
 * This test tries to loads valid world yaml file which in turn tries to
 * load a invalid map yaml file. It should throw an exception.
 */
TEST_F(FlatlandServerLoadWorldTest, map_invalid_A) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/map_invalid_A/world.yaml");
  test_yaml_fail("Missing/invalid \"origin\" in 2d layer");
}

/**
 * This test tries to loads valid world yaml file which in turn load a map yaml
 * file which then inturn tries to load a non-exists map image file. It should
 * throw an exception
 */
TEST_F(FlatlandServerLoadWorldTest, map_invalid_B) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/map_invalid_B/world.yaml");
  test_yaml_fail("Failed to load .*.png");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
