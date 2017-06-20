/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  flat_server_load_world_test.cpp
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

#include <ros/ros.h>
#include <ros/package.h>
#include <gtest/gtest.h>
#include <flatland_server/world.h>
#include <flatland_server/exceptions.h>
#include <Box2D/Box2D.h>
#include <boost/filesystem.hpp>
#include <string>
#include <regex>

namespace fs = boost::filesystem;
using namespace flatland_server;

class FlatlandServerLoadWorldTest : public ::testing::Test
{
protected:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;

  FlatlandServerLoadWorldTest() {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
  }

  void test_yaml_fail(std::string regex_str) {
    try {
      World *w = World::make_world(world_yaml.string());
      w->load_layers(world_yaml.string());
      w->load_models(world_yaml.string());

      delete w;

      FAIL() << "Expected YAMLException, it passed instead";
    }
    catch(YAMLException &e) {
      // do a regex match against error messages
      std::cmatch match;
      std::regex regex(regex_str);
      EXPECT_TRUE(std::regex_match(e.what(), match, regex)) <<
        "'" + std::string(e.what()) + "'" + " did not match against " + "'" + 
        regex_str + "'";
    }
    catch (...) {
      FAIL() << "Expected YAMLException, another exception was caught instead";
    }
  }

  // return the index if found, -1 otherwise
  int does_edge_exist(const b2EdgeShape &edge, 
    const std::vector<std::pair<b2Vec2, b2Vec2>> &edges) {
      for (int i = 0; i < edges.size(); i++) {
        auto e = edges[i];
        if ((edge.m_vertex1.x == e.first.x &&
             edge.m_vertex1.y == e.first.y &&
             edge.m_vertex2.x == e.second.x &&
             edge.m_vertex2.y == e.second.y) ||
            (edge.m_vertex1.x == e.second.x &&
             edge.m_vertex1.y == e.second.y &&
             edge.m_vertex2.x == e.first.x &&
             edge.m_vertex2.y == e.first.y)) {
              return i;
            }
      }
    
    return -1;
  }

  bool does_edges_exactly_match(const std::vector<b2EdgeShape> &edges1,
    const std::vector<std::pair<b2Vec2, b2Vec2>> &edges2) {

      std::vector<std::pair<b2Vec2, b2Vec2>> edges_cpy = edges2;
      for (int i = 0; i < edges1.size(); i++) {
        auto e = edges1[i];
        int ret_idx = does_edge_exist(e, edges_cpy);

        if (ret_idx < 0) {
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
};

// Declare a test
TEST_F(FlatlandServerLoadWorldTest, simple_test_A)
{
  world_yaml = this_file_dir / 
    fs::path("load_world_tests/simple_test_A/world.yaml");
  World *w = World::make_world(world_yaml.string());
  w->load_layers(world_yaml.string());
  w->load_models(world_yaml.string());

  ASSERT_EQ(w->layers_.size(), 2);

  EXPECT_STREQ(w->layers_[0]->name_.c_str(), "2d");
  EXPECT_DOUBLE_EQ(w->layers_[0]->color_[0], 0);
  EXPECT_DOUBLE_EQ(w->layers_[0]->color_[1], 1);
  EXPECT_DOUBLE_EQ(w->layers_[0]->color_[2], 0);
  EXPECT_DOUBLE_EQ(w->layers_[0]->color_[3], 0);
  EXPECT_DOUBLE_EQ(w->layers_[0]->origin_[0], -16.6);
  EXPECT_DOUBLE_EQ(w->layers_[0]->origin_[1], -6.65);
  EXPECT_DOUBLE_EQ(w->layers_[0]->origin_[2], 0.0);
  EXPECT_FALSE(w->layers_[0]->bitmap_.empty());
  EXPECT_EQ(w->layers_[0]->bitmap_.rows, 5);
  EXPECT_EQ(w->layers_[0]->bitmap_.cols, 5);
  EXPECT_DOUBLE_EQ(w->layers_[0]->resolution_, 0.05);
  EXPECT_DOUBLE_EQ(w->layers_[0]->occupied_thresh_, 0.65);
  EXPECT_DOUBLE_EQ(w->layers_[0]->free_thresh_, 0.196);

  EXPECT_STREQ(w->layers_[1]->name_.c_str(), "3d");
  EXPECT_DOUBLE_EQ(w->layers_[1]->color_[0], 1.0);
  EXPECT_DOUBLE_EQ(w->layers_[1]->color_[1], 0.0);
  EXPECT_DOUBLE_EQ(w->layers_[1]->color_[2], 0.0);
  EXPECT_DOUBLE_EQ(w->layers_[1]->color_[3], 0.5);
  EXPECT_DOUBLE_EQ(w->layers_[1]->origin_[0], -3.4e-2);
  EXPECT_DOUBLE_EQ(w->layers_[1]->origin_[1], 200000);
  EXPECT_DOUBLE_EQ(w->layers_[1]->origin_[2], -5e30);
  EXPECT_FALSE(w->layers_[1]->bitmap_.empty());
  EXPECT_EQ(w->layers_[1]->bitmap_.rows, 5);
  EXPECT_EQ(w->layers_[1]->bitmap_.cols, 5);
  EXPECT_DOUBLE_EQ(w->layers_[1]->resolution_, 1.5);
  EXPECT_DOUBLE_EQ(w->layers_[1]->occupied_thresh_, 0.5153);
  EXPECT_DOUBLE_EQ(w->layers_[1]->free_thresh_, 0.2234);

  // check that bitmap vectorization is performed correctly
  std::vector<std::pair<b2Vec2, b2Vec2>> layer0_expected_edges = {
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0, 0), b2Vec2(5, 0)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(1, 1), b2Vec2(4, 1)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(2, 2), b2Vec2(3, 2)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(2, 3), b2Vec2(3, 3)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(1, 4), b2Vec2(4, 4)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0, 5), b2Vec2(5, 5)),

    std::pair<b2Vec2, b2Vec2>(b2Vec2(0, 0), b2Vec2(0, 5)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(1, 1), b2Vec2(1, 4)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(2, 2), b2Vec2(2, 3)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(3, 2), b2Vec2(3, 3)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(4, 1), b2Vec2(4, 4)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(5, 0), b2Vec2(5, 5))
  };

  ASSERT_EQ(w->layers_[0]->extracted_edges.size(), 
    layer0_expected_edges.size());
  EXPECT_TRUE(does_edges_exactly_match(w->layers_[0]->extracted_edges, 
    layer0_expected_edges));

  std::vector<std::pair<b2Vec2, b2Vec2>> layer1_expected_edges = {
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0, 0), b2Vec2(1, 0)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0, 0), b2Vec2(0, 2)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(0, 2), b2Vec2(3, 2)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(3, 2), b2Vec2(3, 4)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(4, 3), b2Vec2(4, 1)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(4, 1), b2Vec2(1, 1)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(1, 0), b2Vec2(1, 1)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(2, 3), b2Vec2(4, 3)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(2, 5), b2Vec2(2, 3)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(1, 4), b2Vec2(3, 4)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(1, 5), b2Vec2(2, 5)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(1, 4), b2Vec2(1, 5)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(4, 4), b2Vec2(5, 4)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(4, 4), b2Vec2(4, 5)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(5, 4), b2Vec2(5, 5)),
    std::pair<b2Vec2, b2Vec2>(b2Vec2(4, 5), b2Vec2(5, 5))
  };

  ASSERT_EQ(w->layers_[1]->extracted_edges.size(), 
    layer1_expected_edges.size());
  EXPECT_TRUE(does_edges_exactly_match(w->layers_[1]->extracted_edges, 
    layer1_expected_edges));

  delete w;
}

TEST_F(FlatlandServerLoadWorldTest, wrong_world_path)
{
  world_yaml = this_file_dir / 
    fs::path("load_world_tests/random_path/world.yaml");
  test_yaml_fail("Error loading.*world.yaml.*bad file");
}

TEST_F(FlatlandServerLoadWorldTest, world_invalid_A)
{
  world_yaml = this_file_dir / 
    fs::path("load_world_tests/world_invalid_A/world.yaml");
  test_yaml_fail("Invalid world param \"properties\"");
}

TEST_F(FlatlandServerLoadWorldTest, world_invalid_B)
{
  world_yaml = this_file_dir / 
    fs::path("load_world_tests/world_invalid_B/world.yaml");
  test_yaml_fail("Invalid \"color\" in 2d layer");
}

TEST_F(FlatlandServerLoadWorldTest, map_invalid_A)
{
  world_yaml = this_file_dir / 
    fs::path("load_world_tests/map_invalid_A/world.yaml");
  test_yaml_fail("Invalid \"origin\" in 2d layer");
}

TEST_F(FlatlandServerLoadWorldTest, map_invalid_B)
{
  world_yaml = this_file_dir / 
    fs::path("load_world_tests/map_invalid_B/world.yaml");
  test_yaml_fail("Failed to load .*.png");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
