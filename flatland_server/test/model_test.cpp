/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	model_test.cpp
 * @brief	Test model methods and functionality
 * @author Joseph Duchesne
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

#include "flatland_server/model.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <string>
#include "flatland_server/collision_filter_registry.h"

// Test the NameSpaceTF method
TEST(TestSuite, testNameSpaceTF) {
  flatland_server::Model has_ns(nullptr, nullptr, std::string("foo"),
                                std::string("has_ns"));
  // namespace "foo" onto tf "bar" => foo_bar
  EXPECT_EQ(has_ns.NameSpaceTF("bar"), "foo_bar");
  // namespace "foo" onto tf "/bar" => bar
  EXPECT_EQ(has_ns.NameSpaceTF("/bar"), "bar");

  flatland_server::Model no_ns(nullptr, nullptr, std::string(""),
                               std::string("no_ns"));
  // namespace "" onto tf "bar" => bar
  EXPECT_EQ(no_ns.NameSpaceTF("bar"), "bar");
  // namespace "" onto tf "/bar" => bar
  EXPECT_EQ(no_ns.NameSpaceTF("/bar"), "bar");
}

// Test the NameSpaceTopic method
TEST(TestSuite, testNameSpaceTopic) {
  flatland_server::Model has_ns(nullptr, nullptr, std::string("foo"),
                                std::string("has_ns"));
  // namespace "foo" onto tf "bar" => foo_bar
  EXPECT_EQ(has_ns.NameSpaceTopic("bar"), "foo/bar");
  // namespace "foo" onto tf "/bar" => bar
  EXPECT_EQ(has_ns.NameSpaceTopic("/bar"), "bar");

  flatland_server::Model no_ns(nullptr, nullptr, std::string(""),
                               std::string("no_ns"));
  // namespace "" onto tf "bar" => bar
  EXPECT_EQ(no_ns.NameSpaceTopic("bar"), "bar");
  // namespace "" onto tf "/bar" => bar
  EXPECT_EQ(no_ns.NameSpaceTopic("/bar"), "bar");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ros::init(argc, argv, "model_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
