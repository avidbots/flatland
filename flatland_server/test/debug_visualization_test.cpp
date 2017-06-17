/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	null.cpp
 * @brief	Sanity check / example test file
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

#include "flatland_server/debug_visualization.h"
#include <Box2D/Box2D.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>

// Test the bodyToMarkers method on a polygon shape
TEST(TestSuite, testBodyToMarkersPolygon) {
  b2Vec2 gravity(0.0, 0.0);
  b2World world(gravity);

  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position.Set(3.0f, 4.0f);
  bodyDef.angle = M_PI_2;
  b2Body* body = world.CreateBody(&bodyDef);

  b2PolygonShape dynamicBox;
  dynamicBox.SetAsBox(1.0f, 2.0f);
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicBox;
  fixtureDef.density = 1.0f;
  fixtureDef.friction = 0.3f;
  body->CreateFixture(&fixtureDef);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::get().bodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.5, 0.7);
  // check that marker was created
  ASSERT_EQ(markers.markers.size(), 1);

  // Check that
  ASSERT_EQ(markers.markers[0].header.frame_id, "map");
  ASSERT_NE(markers.markers[0].header.stamp.sec, 0);
  ASSERT_NE(markers.markers[0].header.stamp.nsec, 0);

  // Check color setting
  ASSERT_NEAR(markers.markers[0].color.r, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].color.g, 0.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].color.b, 0.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].color.a, 0.7, 1e-5);

  // Check position
  ASSERT_NEAR(markers.markers[0].pose.position.x, 3.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.position.y, 4.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.position.z, 0.0, 1e-5);

  // Check orientation
  ASSERT_NEAR(markers.markers[0].pose.orientation.x, 0.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.orientation.y, 0.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.orientation.z, 0.70710678118, 1e-5);
  ASSERT_NEAR(markers.markers[0].pose.orientation.w, 0.70710678118, 1e-5);

  // Check the marker shape
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].LINE_STRIP);
  ASSERT_EQ(markers.markers[0].points.size(),
            5);  // box as line strip: 0, 1, 2, 3, 0
  ASSERT_NEAR(markers.markers[0].points[0].x, -1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[0].y, -2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].x, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].y, -2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[2].x, 1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[2].y, 2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[3].x, -1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[3].y, 2.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[4].x, -1.0, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[4].y, -2.0, 1e-5);
}

// Test the bodyToMarkers method on a circle shape
TEST(TestSuite, testBodyToMarkersCircle) {
  b2Vec2 gravity(0.0, 0.0);
  b2World world(gravity);

  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position.Set(3.0f, 4.0f);
  bodyDef.angle = M_PI_2;
  b2Body* body = world.CreateBody(&bodyDef);

  b2FixtureDef fixtureDef;
  b2CircleShape circle;
  circle.m_p.Set(2.0f, 3.0f);
  circle.m_radius = 0.2f;
  fixtureDef.shape = &circle;
  body->CreateFixture(&fixtureDef);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::get().bodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  // check that marker was created
  ASSERT_EQ(markers.markers.size(), 1);

  // Check the marker shape
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].CYLINDER);
  ASSERT_NEAR(markers.markers[0].scale.x, 0.4, 1e-5);
  ASSERT_NEAR(markers.markers[0].scale.y, 0.4, 1e-5);
  ASSERT_NEAR(markers.markers[0].scale.z, 0.01, 1e-5);
}

// Test the bodyToMarkers method on a edge shape
TEST(TestSuite, testBodyToMarkersEdge) {
  b2Vec2 gravity(0.0, 0.0);
  b2World world(gravity);

  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position.Set(3.0f, 4.0f);
  bodyDef.angle = M_PI_2;
  b2Body* body = world.CreateBody(&bodyDef);

  b2FixtureDef fixtureDef;
  b2EdgeShape edge;
  edge.m_vertex1.Set(0.5, 1.5);
  edge.m_vertex2.Set(3.5, 2.0);
  fixtureDef.shape = &edge;
  body->CreateFixture(&fixtureDef);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::get().bodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  // check that marker was created
  ASSERT_EQ(markers.markers.size(), 1);

  // Check the marker shape
  ASSERT_EQ(markers.markers[0].type, markers.markers[0].LINE_LIST);
  ASSERT_EQ(markers.markers[0].points.size(), 2);
  ASSERT_NEAR(markers.markers[0].points[0].x, 0.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[0].y, 1.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].x, 3.5, 1e-5);
  ASSERT_NEAR(markers.markers[0].points[1].y, 2.0, 1e-5);
}

// Test the bodyToMarkers method on an unsupported shape
TEST(TestSuite, testBodyToMarkersUnsupported) {
  b2Vec2 gravity(0.0, 0.0);
  b2World world(gravity);

  b2BodyDef bodyDef;
  b2Body* body = world.CreateBody(&bodyDef);

  b2FixtureDef fixtureDef;
  b2Vec2 vs[4];
  vs[0].Set(1.7f, 0.0f);
  vs[1].Set(1.0f, 0.25f);
  vs[2].Set(0.0f, 0.0f);
  vs[3].Set(-1.7f, 0.4f);
  b2ChainShape chain;
  chain.CreateChain(vs, 4);
  fixtureDef.shape = &chain;
  body->CreateFixture(&fixtureDef);

  visualization_msgs::MarkerArray markers;
  flatland_server::DebugVisualization::get().bodyToMarkers(markers, body, 1.0,
                                                           0.0, 0.0, 1.0);
  // check that marker was not created
  ASSERT_EQ(markers.markers.size(), 0);
}

// Todo: test body with multiple fixtures
// Todo: test multiple bodies

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  ros::init(argc, argv, "debug_visualization_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
