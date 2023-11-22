/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	geometry_test.cpp
 * @brief	Test geometry
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

#include "flatland_server/geometry.h"

#include <gtest/gtest.h>

#include <cmath>

// Test the CreateTransform method
TEST(TestSuite, testCreateTransform)
{
  flatland_server::RotateTranslate rt = flatland_server::Geometry::CreateTransform(2.0, 1.0, 1.1);

  EXPECT_NEAR(rt.dx, 2.0, 1e-5);
  EXPECT_NEAR(rt.dy, 1.0, 1e-5);
  EXPECT_NEAR(rt.cos, cosf(1.1), 1e-5);
  EXPECT_NEAR(rt.sin, sinf(1.1), 1e-5);
}

// Test the Transform method, translation
TEST(TestSuite, testTransformTranslate)
{
  flatland_server::RotateTranslate rt = flatland_server::Geometry::CreateTransform(2.0, 1.5, 0.0);

  b2Vec2 in(1.0, 2.0);
  b2Vec2 out = flatland_server::Geometry::Transform(in, rt);

  EXPECT_NEAR(out.x, 3.0, 1e-5);
  EXPECT_NEAR(out.y, 3.5, 1e-5);
}

// Test the Transform method, rotation
TEST(TestSuite, testTransformRotate)
{
  flatland_server::RotateTranslate rt =
    flatland_server::Geometry::CreateTransform(0.0, 0.0, M_PI_2);

  b2Vec2 in(1.0, 2.0);
  b2Vec2 out = flatland_server::Geometry::Transform(in, rt);

  EXPECT_NEAR(out.x, -2.0, 1e-5);
  EXPECT_NEAR(out.y, 1.0, 1e-5);
}

// Test the Transform method, translation + rotation
TEST(TestSuite, testTransformTranslateAndRotate)
{
  flatland_server::RotateTranslate rt = flatland_server::Geometry::CreateTransform(1.0, 0.5, -M_PI);

  b2Vec2 in(-1.0, 1.5);
  b2Vec2 out = flatland_server::Geometry::Transform(in, rt);

  EXPECT_NEAR(out.x, 2.0, 1e-5);
  EXPECT_NEAR(out.y, -1.0, 1e-5);
}

// Run all the tests that were declared with TEST()
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
