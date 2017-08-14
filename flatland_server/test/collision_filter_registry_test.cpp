/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  collision_filter_registry_test.cpp
 * @brief Testing collision filter registry
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

#include <flatland_server/collision_filter_registry.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>

using namespace flatland_server;

typedef CollisionFilterRegistry CFR;

class CollisionFilterRegistryTest : public ::testing::Test {
 protected:
  CollisionFilterRegistry cfr;
};

TEST_F(CollisionFilterRegistryTest, empty_test) {
  EXPECT_FALSE(cfr.IsLayersFull());
  EXPECT_EQ(cfr.LookUpLayerId("random_layer"), CFR::LAYER_NOT_EXIST);

  std::vector<std::string> layer_names = cfr.GetAllLayers();
  EXPECT_EQ(layer_names.size(), 0);
}

TEST_F(CollisionFilterRegistryTest, register_collide_test) {
  EXPECT_EQ(cfr.RegisterCollide(), 1);
  EXPECT_EQ(cfr.RegisterCollide(), 2);
  EXPECT_EQ(cfr.RegisterCollide(), 3);
  EXPECT_EQ(cfr.RegisterCollide(), 4);
  EXPECT_EQ(cfr.RegisterCollide(), 5);
}

TEST_F(CollisionFilterRegistryTest, register_no_collide_test) {
  EXPECT_EQ(cfr.RegisterNoCollide(), -1);
  EXPECT_EQ(cfr.RegisterNoCollide(), -2);
  EXPECT_EQ(cfr.RegisterNoCollide(), -3);
  EXPECT_EQ(cfr.RegisterNoCollide(), -4);
  EXPECT_EQ(cfr.RegisterNoCollide(), -5);
}

TEST_F(CollisionFilterRegistryTest, register_layers_test) {
  EXPECT_EQ(CFR::MAX_LAYERS, 16);

  EXPECT_EQ(cfr.RegisterLayer("layer1"), 0);
  EXPECT_EQ(cfr.RegisterLayer("layer2"), 1);
  EXPECT_EQ(cfr.RegisterLayer("layer3"), 2);
  EXPECT_EQ(cfr.RegisterLayer("layer4"), 3);
  EXPECT_EQ(cfr.RegisterLayer("layer5"), 4);

  EXPECT_EQ(cfr.LookUpLayerId("layer1"), 0);
  EXPECT_EQ(cfr.LookUpLayerId("layer2"), 1);
  EXPECT_EQ(cfr.LookUpLayerId("layer3"), 2);
  EXPECT_EQ(cfr.LookUpLayerId("layer4"), 3);
  EXPECT_EQ(cfr.LookUpLayerId("layer5"), 4);

  std::vector<std::string> layer_names = cfr.GetAllLayers();
  EXPECT_EQ(layer_names.size(), 5);

  EXPECT_TRUE(std::find(layer_names.begin(), layer_names.end(), "layer1") !=
              layer_names.end());
  EXPECT_TRUE(std::find(layer_names.begin(), layer_names.end(), "layer2") !=
              layer_names.end());
  EXPECT_TRUE(std::find(layer_names.begin(), layer_names.end(), "layer3") !=
              layer_names.end());
  EXPECT_TRUE(std::find(layer_names.begin(), layer_names.end(), "layer4") !=
              layer_names.end());
  EXPECT_TRUE(std::find(layer_names.begin(), layer_names.end(), "layer5") !=
              layer_names.end());

  EXPECT_EQ(cfr.RegisterLayer("layer5"), CFR::LAYER_ALREADY_EXIST);
  EXPECT_EQ(cfr.RegisterLayer("layer6"), 5);
  EXPECT_EQ(cfr.RegisterLayer("layer7"), 6);
  EXPECT_EQ(cfr.RegisterLayer("layer8"), 7);
  EXPECT_EQ(cfr.RegisterLayer("layer9"), 8);
  EXPECT_EQ(cfr.RegisterLayer("layer10"), 9);
  EXPECT_EQ(cfr.RegisterLayer("layer11"), 10);
  EXPECT_EQ(cfr.RegisterLayer("layer12"), 11);
  EXPECT_EQ(cfr.RegisterLayer("layer13"), 12);
  EXPECT_EQ(cfr.RegisterLayer("layer14"), 13);
  EXPECT_EQ(cfr.RegisterLayer("layer15"), 14);
  EXPECT_EQ(cfr.RegisterLayer("layer16"), 15);
  EXPECT_EQ(cfr.RegisterLayer("layer17"), CFR::LAYERS_FULL);

  EXPECT_TRUE(cfr.IsLayersFull());
  layer_names = cfr.GetAllLayers();

  EXPECT_EQ(layer_names.size(), 16);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
