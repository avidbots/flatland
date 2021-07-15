/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2021 Avidbots Corp.
 * @name  dynamics_limits_test.cpp
 * @brief Test dynamics limits class
 * @author Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Avidbots Corp.
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

#include <flatland_plugins/dynamics_limits.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>
#include <gtest/gtest.h>

using namespace flatland_plugins;

/**
 * Test loading blank parameters
 */
TEST(DynamicsLimitsTest, test_Configure_blank) {
  auto dynamics_limits = DynamicsLimits();

  // Ensure defaults are zero
  EXPECT_NEAR(0.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.velocity_limit_, 1e-3);
}


/**
 * Test loading blank parameters
 */
TEST(DynamicsLimitsTest, test_Configure_empty_yaml) {
  auto dynamics_limits = DynamicsLimits();
  YAML::Node config = YAML::Node();
  dynamics_limits.Configure(config);

  // Ensure defaults are zero
  EXPECT_NEAR(0.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.velocity_limit_, 1e-3);
}


/**
 * Test loading acceleration+velocity parameters
 */
TEST(DynamicsLimitsTest, test_Configure_two_params) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 0.1;
  config["velocity_limit"] = 2.0;
  dynamics_limits.Configure(config);

  // Ensure defaults are zero
  EXPECT_NEAR(0.1, dynamics_limits.acceleration_limit_, 1e-3);
  // deceleration cap defaults to accleration cap if not set
  EXPECT_NEAR(0.1, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(2.0, dynamics_limits.velocity_limit_, 1e-3);
}

/**
 * Test loading acceleration, deceleration, velocity parameters
 */
TEST(DynamicsLimitsTest, test_Configure_three_params) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 0.1;
  config["deceleration_limit"] = 0.2;
  config["velocity_limit"] = 2.0;
  dynamics_limits.Configure(config);

  // Ensure params are correct
  EXPECT_NEAR(0.1, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(0.2, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(2.0, dynamics_limits.velocity_limit_, 1e-3);
}

// todo: test Saturate

/**
 * Test Limit function without dynamics limits in place
 */
TEST(DynamicsLimitsTest, test_Limit_noop) {
  auto dynamics_limits = DynamicsLimits();

  double result;
  result = dynamics_limits.Limit(0, 1.0, 0.05);
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(1.0, 1.0, 0.05);
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(0.1, 1.0, 0.05);
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(-100, 1.0, 0.1);
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(-100, -1.0, 0.1);
  EXPECT_NEAR(-1.0, result, 1e-3);
  result = dynamics_limits.Limit(-100, -21.0, 0.1);
  EXPECT_NEAR(-21.0, result, 1e-3);
}


/**
 * Test Limit function
 */
TEST(DynamicsLimitsTest, test_Limit_velocity) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["velocity_limit"] = 1.0;
  dynamics_limits.Configure(config);

  EXPECT_NEAR(0.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(1.0, dynamics_limits.velocity_limit_, 1e-3);

  double result;
  result = dynamics_limits.Limit(0, 1.0, 0.05);
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(1.0, 1.0, 0.05);
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(0, 2.0, 0.05);
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(0, -1.0, 0.1);
  EXPECT_NEAR(-1.0, result, 1e-3);
  result = dynamics_limits.Limit(0, -0.5, 0.1);
  EXPECT_NEAR(-0.5, result, 1e-3);
  result = dynamics_limits.Limit(1.0, -0.5, 0.1);
  EXPECT_NEAR(-0.5, result, 1e-3);
  result = dynamics_limits.Limit(-100, -1.0, 0.1);
  EXPECT_NEAR(-1.0, result, 1e-3);
  result = dynamics_limits.Limit(0, -3.0, 0.1);
  EXPECT_NEAR(-1.0, result, 1e-3);
}


/**
 * Test Limit function wrt. acceleration
 */
TEST(DynamicsLimitsTest, test_Limit_acceleration) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 9.0;
  dynamics_limits.Configure(config);

  EXPECT_NEAR(9.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(9.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.velocity_limit_, 1e-3);

  double result;
  result = dynamics_limits.Limit(0, 1.0, 0.05);  // Try to accelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(0.45, result, 1e-3);
  result = dynamics_limits.Limit(1.0, 0.0, 0.05);  // Try to decelerate at -20m/s/s, when the limit is 9
  EXPECT_NEAR(0.55, result, 1e-3);
  result = dynamics_limits.Limit(10.0, 0.0, 0.05);  // Try to decelerate at -200m/s/s, when the limit is 9
  EXPECT_NEAR(9.55, result, 1e-3);
  result = dynamics_limits.Limit(-10.0, 0.0, 0.05);  // Try to decelerate at 200m/s/s, when the limit is 9
  EXPECT_NEAR(-9.55, result, 1e-3);
  result = dynamics_limits.Limit(5, -5, 0.05);  // Try to decelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(4.55, result, 1e-3);
  result = dynamics_limits.Limit(-3, -2, 0.05);  // Try to decelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(-2.55, result, 1e-3);
}

TEST(DynamicsLimitsTest, test_Saturate) {
  auto dynamics_limits = DynamicsLimits();
  double result;
  // Useless, strict bounds
  result = DynamicsLimits::Saturate(0, 0, 0);  // Bound between 0 and 0, 0
  EXPECT_NEAR(0, result, 1e-3);
  result = DynamicsLimits::Saturate(1, 0, 0);  // Bound between 1 and 0, 0
  EXPECT_NEAR(0, result, 1e-3);
  result = DynamicsLimits::Saturate(-1, 0, 0);  // Bound between -1 and 0, 0
  EXPECT_NEAR(0, result, 1e-3);

  // Valid bounds
  result = DynamicsLimits::Saturate(0, -1.1, 2.2);
  EXPECT_NEAR(0, result, 1e-3);
  result = DynamicsLimits::Saturate(1, -1.1, 2.2);
  EXPECT_NEAR(1, result, 1e-3);
  result = DynamicsLimits::Saturate(-1,  -1.1, 2.2);
  EXPECT_NEAR(-1, result, 1e-3);
  result = DynamicsLimits::Saturate(3, -1.1, 2.2);
  EXPECT_NEAR(2.2, result, 1e-3);
  result = DynamicsLimits::Saturate(-3,  -1.1, 2.2);
  EXPECT_NEAR(-1.1, result, 1e-3);

  // Invalid bounds, input not changed
  result = DynamicsLimits::Saturate(0, 1.1, -2.2);
  EXPECT_NEAR(0, result, 1e-3);
  result = DynamicsLimits::Saturate(1, 1.1, -2.2);
  EXPECT_NEAR(1, result, 1e-3);
  result = DynamicsLimits::Saturate(-1,  1.1, -2.2);
  EXPECT_NEAR(-1, result, 1e-3);
  result = DynamicsLimits::Saturate(3, 1.1, -2.2);
  EXPECT_NEAR(3, result, 1e-3);
  result = DynamicsLimits::Saturate(-3,  1.1, -2.2);
  EXPECT_NEAR(-3, result, 1e-3);
}

/**
 * Test Limit function wrt. acceleration and decelleration
 */
TEST(DynamicsLimitsTest, test_Limit_deceleration) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 1.0;
  config["deceleration_limit"] = 9.0;
  dynamics_limits.Configure(config);

  EXPECT_NEAR(1.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(9.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.velocity_limit_, 1e-3);

  double result;
  result = dynamics_limits.Limit(0, 1.0, 0.05);  // Try to accelerate at 20m/s/s, when the limit is 1 (acceleration)
  EXPECT_NEAR(0.05, result, 1e-3);
  result = dynamics_limits.Limit(0.97, 1.0, 0.05);  // Try to accelerate at 1m/s/s, when the limit is 1
  EXPECT_NEAR(1.0, result, 1e-3);
  result = dynamics_limits.Limit(1.0, 0.0, 0.05);  // Try to decelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(0.55, result, 1e-3);
  result = dynamics_limits.Limit(-1.0, -2.0, 0.05);  // Try to acccelerate at 20m/s/s, when the limit is 1
  EXPECT_NEAR(-1.05, result, 1e-3);
  result = dynamics_limits.Limit(-2.0, -1.0, 0.05);  // Try to decelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(-1.55, result, 1e-3);
  result = dynamics_limits.Limit(-2.0, 1.0, 0.05);  // Try to decelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(-1.55, result, 1e-3);
  result = dynamics_limits.Limit(-1.0, 2.0, 0.05);  // Try to decelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(-0.55, result, 1e-3);
  result = dynamics_limits.Limit(1.0, 3.0, 0.05);  // Try to acccelerate at 20m/s/s, when the limit is 1
  EXPECT_NEAR(1.05, result, 1e-3);
  result = dynamics_limits.Limit(3.0, -1.0, 0.05);  // Try to decelerate at 20m/s/s, when the limit is 9
  EXPECT_NEAR(2.55, result, 1e-3);
}

/**
 * Test Limit function wrt. acceleration and decelleration
 */
TEST(DynamicsLimitsTest, test_Limit_zero_crossing) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 1.0;
  config["deceleration_limit"] = 2.0;
  dynamics_limits.Configure(config);

  EXPECT_NEAR(1.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(2.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.velocity_limit_, 1e-3);

  double result;
  result = dynamics_limits.Limit(2.0, 0.0, 0.05);  // Decelerate, without zero crossing inside the timestep
  EXPECT_NEAR(1.9, result, 1e-3);

  result = dynamics_limits.Limit(0.08, -1.0, 0.05);  // Decelerate, with zero crossing inside the timestep
  // the deceletion from 0.08 -> 0m/s is done at 2m/s/s, taking 0.04s, 80% of the timestep
  // then the acceleration from 0 -> -1 has the accleration constraints applied given the remainder of the timestep (0.01s)
  // resulting in an effective acceleration limit of 1.8m/s/s over the timestep
  EXPECT_NEAR(-0.01, result, 1e-3);
}


/**
 * Test Limit function wrt. acceleration and decelleration corner case around zero crossing
 */
TEST(DynamicsLimitsTest, test_Limit_zero_crossing_corner_case1) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 0.0;
  config["deceleration_limit"] = 2.0;
  dynamics_limits.Configure(config);

  EXPECT_NEAR(0.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(2.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.velocity_limit_, 1e-3);

  double result;
  result = dynamics_limits.Limit(2.0, 0.0, 0.05);  // Decelerate, without zero crossing inside the timestep
  EXPECT_NEAR(1.9, result, 1e-3);

  result = dynamics_limits.Limit(0.08, -1.0, 0.05);  // Decelerate, with zero crossing inside the timestep
  // The first 80% decelerates at 2m/s/s, then the remaining 20% of the timestep accelerates from 0 to -1m/s without acceleration limit
  EXPECT_NEAR(-1.0, result, 1e-3);
}

/**
 * Test Limit function wrt. acceleration and decelleration corner case around zero crossing
 */
TEST(DynamicsLimitsTest, test_Limit_zero_crossing_corner_case2) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 2.0;
  config["deceleration_limit"] = 0.0;
  dynamics_limits.Configure(config);

  EXPECT_NEAR(2.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.0, dynamics_limits.velocity_limit_, 1e-3);

  double result;
  result = dynamics_limits.Limit(2.0, 0.0, 0.05);  // Decelerate, without zero crossing inside the timestep
  EXPECT_NEAR(0.0, result, 1e-3);

  result = dynamics_limits.Limit(1.0, -1.0, 0.05);  // Decelerate, with zero crossing inside the timestep (no limit)
  // The portion from 1m/s to 0m/s has no limit, however the timestep is used to accelerate under limits
  // 0.05 sec at -2m/s/s acceleration = -0.1m/s final velocity
  EXPECT_NEAR(-0.1, result, 1e-3);
}

/**
 * Test Limit function wrt. acceleration asnd decelleration
 */
TEST(DynamicsLimitsTest, test_Limit_acceleration_and_velocity) {
  auto dynamics_limits = DynamicsLimits();

  YAML::Node config = YAML::Node();
  config["acceleration_limit"] = 1.0;
  config["deceleration_limit"] = 9.0;
  config["velocity_limit"] = 0.5;
  dynamics_limits.Configure(config);

  EXPECT_NEAR(1.0, dynamics_limits.acceleration_limit_, 1e-3);
  EXPECT_NEAR(9.0, dynamics_limits.deceleration_limit_, 1e-3);
  EXPECT_NEAR(0.5, dynamics_limits.velocity_limit_, 1e-3);

  double result;
  result = dynamics_limits.Limit(0, 1.0, 0.05);  // Try to accelerate at 20m/s/s, when the limit is 1 (acceleration)
  EXPECT_NEAR(0.05, result, 1e-3);

  result = dynamics_limits.Limit(0.48, 1.48, 0.05);  // Same, but hit the velocity limit
  EXPECT_NEAR(0.5, result, 1e-3);

  result = dynamics_limits.Limit(1.0, 2.0, 0.05);  // Same, but we're already over the velocity limit
  // The invalid input temporarily results in invalid output, because we have to voilate either decelleration  or velocity constraints
  // In this case, we violate velocity constraints
  EXPECT_NEAR(0.55, result, 1e-3);

  result = dynamics_limits.Limit(0.55, 2.0, 0.05);  // Same, but we're already over the velocity limit, 
  // but this time we can achieve velocity constraints without violating decelleration constraints
  EXPECT_NEAR(0.5, result, 1e-3);
}



// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
