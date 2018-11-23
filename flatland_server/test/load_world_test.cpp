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
#include <flatland_server/debug_visualization.h>
#include <flatland_server/entity.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/geometry.h>
#include <flatland_server/types.h>
#include <flatland_server/world.h>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <regex>
#include <string>

namespace fs = boost::filesystem;
using namespace flatland_server;

class LoadWorldTest : public ::testing::Test {
 protected:
  boost::filesystem::path this_file_dir;
  boost::filesystem::path world_yaml;
  World *w;

  void SetUp() override {
    this_file_dir = boost::filesystem::path(__FILE__).parent_path();
    w = nullptr;
  }

  void TearDown() override {
    if (w != nullptr) {
      delete w;
    }
  }

  // to test that the world instantiation will fail, and the exception
  // message matches the given regex string
  void test_yaml_fail(std::string regex_str) {
    // do a regex match against error messages
    std::cmatch match;
    std::regex regex(regex_str);

    try {
      w = World::MakeWorld(world_yaml.string());
      ADD_FAILURE() << "Expected an exception, but none were raised";
    } catch (const Exception &e) {
      EXPECT_TRUE(std::regex_match(e.what(), match, regex))
          << "Exception Message '" + std::string(e.what()) + "'" +
                 " did not match against regex '" + regex_str + "'";
    } catch (const std::exception &e) {
      ADD_FAILURE() << "Expected flatland_server::Exception, another exception "
                       "was caught instead: "
                    << e.what();
    }
  }

  bool float_cmp(double n1, double n2) {
    bool ret = fabs(n1 - n2) < 1e-5;
    return ret;
  }

  // return the id if found, -1 otherwise
  int does_edge_exist(const b2EdgeShape &edge,
                      const std::vector<std::pair<b2Vec2, b2Vec2>> &edges) {
    for (unsigned int i = 0; i < edges.size(); i++) {
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
    for (unsigned int i = 0; i < edges1.size(); i++) {
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

  bool ColorEq(const Color &c1, const Color &c2) {
    if (c1 != c2) {
      printf("Color Actual:[%f,%f,%f,%f] != Expected:[%f,%f,%f,%f]\n", c1.r,
             c1.g, c1.b, c1.a, c2.r, c2.g, c2.b, c2.a);
      return false;
    }
    return true;
  }

  bool BodyEq(Body *body, const std::string name, b2BodyType type,
              const std::array<double, 3> &pose,
              const std::array<double, 4> &color, double linear_damping,
              double angular_damping) {
    b2Vec2 t = body->physics_body_->GetPosition();
    double a = body->physics_body_->GetAngle();

    if (name != body->name_) {
      printf("Name Actual:%s != Expected:%s\n", body->name_.c_str(),
             name.c_str());
      return false;
    }

    if (type != body->physics_body_->GetType()) {
      /*
      enum b2BodyType
      {
        b2_staticBody = 0,
        b2_kinematicBody,
        b2_dynamicBody
      };
      */
      printf("Body type Actual:%d != Expected:%d\n",
             body->physics_body_->GetType(), type);
      return false;
    }

    if (!float_cmp(t.x, pose[0]) || !float_cmp(t.y, pose[1]) ||
        !float_cmp(a, pose[2])) {
      printf("Pose Actual:[%f,%f,%f] != Expected:[%f,%f,%f]\n", t.x, t.y, a,
             pose[0], pose[1], pose[2]);
      return false;
    }

    if (!ColorEq(body->color_, Color(color))) {
      return false;
    }

    if (!float_cmp(linear_damping, body->physics_body_->GetLinearDamping())) {
      printf("Linear Damping Actual:%f != Expected:%f\n",
             body->physics_body_->GetLinearDamping(), linear_damping);
      return false;
    }

    if (!float_cmp(angular_damping, body->physics_body_->GetAngularDamping())) {
      printf("Angular Damping Actual %f != Expected:%f\n",
             body->physics_body_->GetAngularDamping(), angular_damping);
      return false;
    }

    return true;
  }

  bool CircleEq(b2Fixture *f, double x, double y, double r) {
    if (f->GetShape()->GetType() != b2Shape::e_circle) {
      /*
      enum Type
      {
        e_circle = 0,
        e_edge = 1,
        e_polygon = 2,
        e_chain = 3,
        e_typeCount = 4
      };
      */
      printf("Shape is not of type b2Shape::e_circle, Actual=%d\n",
             f->GetShape()->GetType());
      return false;
    }

    b2CircleShape *s = dynamic_cast<b2CircleShape *>(f->GetShape());

    if (!float_cmp(r, s->m_radius) || !float_cmp(x, s->m_p.x) ||
        !float_cmp(y, s->m_p.y)) {
      printf("Actual:[x=%f,y=%f,r=%f] != Expected:[%f,%f,%f] \n", s->m_p.x,
             s->m_p.y, s->m_radius, x, y, r);
      return false;
    }
    return true;
  }

  bool PolygonEq(b2Fixture *f, std::vector<std::pair<double, double>> points) {
    if (f->GetShape()->GetType() != b2Shape::e_polygon) {
      /*
      enum Type
      {
        e_circle = 0,
        e_edge = 1,
        e_polygon = 2,
        e_chain = 3,
        e_typeCount = 4
      };
      */
      printf("Shape is not of type b2Shape::e_polygon, Actual=%d\n",
             f->GetShape()->GetType());
      return false;
    }

    b2PolygonShape *s = dynamic_cast<b2PolygonShape *>(f->GetShape());
    int cnt = s->m_count;
    if (cnt != points.size()) {
      printf("Number of points Actual:%d != Expected:%lu\n", cnt,
             points.size());
      return false;
    }

    auto pts = points;

    for (unsigned int i = 0; i < cnt; i++) {
      const b2Vec2 p = s->m_vertices[i];

      bool found_match = false;
      int j;
      for (j = 0; j < pts.size(); j++) {
        if (!float_cmp(p.x, points[i].first) ||
            !float_cmp(p.y, points[i].second)) {
          found_match = true;
          break;
        }
      }

      if (!found_match) {
        // cannot find a matching point, print the expected and actual points
        printf("Actual: [");
        for (int k = 0; k < cnt; k++) {
          printf("[%f,%f],", s->m_vertices[k].x, s->m_vertices[k].y);
        }

        printf("] != Expected: [");
        for (int k = 0; k < points.size(); k++) {
          printf("[%f,%f],", points[k].first, points[k].second);
        }
        printf("]\n");

        return false;
      }

      pts.erase(pts.begin() + j);
    }

    if (pts.size() == 0) {
      return true;
    } else {
      return false;
    }
  }

  std::vector<b2Fixture *> GetBodyFixtures(Body *body) {
    std::vector<b2Fixture *> fixtures;

    b2Body *b = body->physics_body_;
    for (b2Fixture *f = b->GetFixtureList(); f; f = f->GetNext()) {
      fixtures.push_back(f);
    }

    std::reverse(fixtures.begin(), fixtures.end());

    return fixtures;
  }

  bool FixtureEq(b2Fixture *f, bool is_sensor, int group_index,
                 uint16_t category_bits, uint16_t mask_bits, double density,
                 double friction, double restitution) {
    if (f->IsSensor() != is_sensor) {
      printf("is_sensor Actual:%d != Expected:%d\n", f->IsSensor(), is_sensor);
      return false;
    }

    if (f->GetFilterData().groupIndex != group_index) {
      printf("group_index Actual:%d != Expected:%d\n",
             f->GetFilterData().groupIndex, group_index);
      return false;
    }

    if (f->GetFilterData().categoryBits != category_bits) {
      printf("category_bits Actual:0x%X != Expected:0x%X\n",
             f->GetFilterData().categoryBits, category_bits);
      return false;
    }

    if (f->GetFilterData().maskBits != mask_bits) {
      printf("mask_bits Actual:0x%X != Expected:0x%X\n",
             f->GetFilterData().maskBits, mask_bits);
      return false;
    }

    if (!float_cmp(f->GetDensity(), density)) {
      printf("density Actual:%f != Expected:%f\n", f->GetDensity(), density);
      return false;
    }

    if (!float_cmp(f->GetFriction(), friction)) {
      printf("friction Actual:%f != Expected:%f\n", f->GetFriction(), friction);
      return false;
    }

    if (!float_cmp(f->GetRestitution(), restitution)) {
      printf("restitution Actual:%f != Expected:%f\n", f->GetRestitution(),
             restitution);
      return false;
    }

    return true;
  }

  bool JointEq(Joint *joint, const std::string name,
               const std::array<double, 4> &color, Body *body_A,
               const std::array<double, 2> &anchor_A, Body *body_B,
               const std::array<double, 2> &anchor_B, bool collide_connected) {
    b2Joint *j = joint->physics_joint_;

    if (name != joint->name_) {
      printf("Name Actual:%s != Expected:%s\n", joint->name_.c_str(),
             name.c_str());
      return false;
    }

    if (!ColorEq(joint->color_, color)) {
      return false;
    }

    if (j->GetBodyA() != body_A->physics_body_) {
      printf("BodyA ptr Actual %p != Expected:%p\n",
             joint->physics_joint_->GetBodyA(), body_A->physics_body_);
      return false;
    }

    if (j->GetBodyB() != body_B->physics_body_) {
      printf("BodyB ptr Actual %p != Expected:%p\n", j->GetBodyB(),
             body_B->physics_body_);
      return false;
    }

    // GetAnchor returns world coordinates, we want to verify against
    // local coordinates
    b2Vec2 local_anchor_A = j->GetAnchorA() - j->GetBodyA()->GetPosition();
    b2Vec2 local_anchor_B = j->GetAnchorB() - j->GetBodyB()->GetPosition();

    if (!float_cmp(local_anchor_A.x, anchor_A[0]) ||
        !float_cmp(local_anchor_A.y, anchor_A[1])) {
      printf("Anchor A Actual:[%f,%f] != Expected:[%f,%f]\n", local_anchor_A.x,
             local_anchor_A.y, anchor_A[0], anchor_A[1]);
      return false;
    }

    if (!float_cmp(local_anchor_B.x, anchor_B[0]) ||
        !float_cmp(local_anchor_B.y, anchor_B[1])) {
      printf("Anchor B Actual:[%f,%f] != Expected:[%f,%f]\n", local_anchor_B.x,
             local_anchor_B.y, anchor_B[0], anchor_B[1]);
      return false;
    }

    if (collide_connected != j->GetCollideConnected()) {
      printf("Collide connected Actual:%d != Expected:%d\n",
             j->GetCollideConnected(), collide_connected);
      return false;
    }

    return true;
  }

  bool WeldEq(Joint *joint, double angle, double freq, double damping) {
    b2WeldJoint *j = dynamic_cast<b2WeldJoint *>(joint->physics_joint_);

    if (j->GetType() != e_weldJoint) {
      /*
      enum b2JointType
      {
        e_unknownJoint, --> C++ should defaults initialize at zero?
        e_revoluteJoint,
        e_prismaticJoint,
        e_distanceJoint,
        e_pulleyJoint,
        e_mouseJoint,
        e_gearJoint,
        e_wheelJoint,
        e_weldJoint,
        e_frictionJoint,
        e_ropeJoint,
        e_motorJoint
      };
      */
      printf("Joint type Actual:%d != Expected:%d(weld joint)\n", j->GetType(),
             e_weldJoint);
      return false;
    }

    if (!float_cmp(angle, j->GetReferenceAngle())) {
      printf("Angle Actual:%f != Expected:%f\n", angle, j->GetReferenceAngle());
      return false;
    }

    if (!float_cmp(freq, j->GetFrequency())) {
      printf("Frequency Actual:%f != Expected:%f\n", freq, j->GetFrequency());
      return false;
    }

    if (!float_cmp(damping, j->GetDampingRatio())) {
      printf("Damping Actual:%f != Expected:%f\n", damping,
             j->GetDampingRatio());
      return false;
    }

    return true;
  }

  bool RevoluteEq(Joint *joint, bool is_limit_enabled,
                  const std::array<double, 2> limits) {
    b2RevoluteJoint *j = dynamic_cast<b2RevoluteJoint *>(joint->physics_joint_);

    if (j->GetType() != e_revoluteJoint) {
      /*
      enum b2JointType
      {
        e_unknownJoint, --> C++ defaults initialize at zero?
        e_revoluteJoint,
        e_prismaticJoint,
        e_distanceJoint,
        e_pulleyJoint,
        e_mouseJoint,
        e_gearJoint,
        e_wheelJoint,
        e_weldJoint,
        e_frictionJoint,
        e_ropeJoint,
        e_motorJoint
      };
      */
      printf("Joint type Actual:%d != Expected:%d(revolute joint)\n",
             j->GetType(), e_revoluteJoint);
      return false;
    }

    if (is_limit_enabled != j->IsLimitEnabled()) {
      printf("Limits enabled Actual:%d != Expected:%d\n", is_limit_enabled,
             j->IsLimitEnabled());
      return false;
    }

    if (is_limit_enabled && (!float_cmp(limits[0], j->GetLowerLimit()) ||
                             !float_cmp(limits[1], j->GetUpperLimit()))) {
      printf("Limits Actual:[%f,%f] != Expected:[%f,%f]\n", j->GetLowerLimit(),
             j->GetLowerLimit(), limits[0], limits[1]);
      return false;
    }

    return true;
  }
};

/**
 * This test loads the world, layers, models from the given world
 * yaml file and checks that all configurations, data, and calculations are
 * correct after instantiation
 */
TEST_F(LoadWorldTest, simple_test_A) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/simple_test_A/world.yaml");
  w = World::MakeWorld(world_yaml.string());

  EXPECT_EQ(w->physics_velocity_iterations_, 11);
  EXPECT_EQ(w->physics_position_iterations_, 12);

  ASSERT_EQ(w->layers_.size(), 4);

  // check that layer 0 settings are loaded correctly
  EXPECT_STREQ(w->layers_[0]->name_.c_str(), "2d");
  ASSERT_EQ(w->layers_[0]->names_.size(), 1);
  EXPECT_STREQ(w->layers_[0]->names_[0].c_str(), "2d");
  EXPECT_EQ(w->layers_[0]->Type(), Entity::EntityType::LAYER);
  EXPECT_TRUE(BodyEq(w->layers_[0]->body_, "2d", b2_staticBody,
                     {0.05, -0.05, 1.57}, {0, 1, 0, 0.675}, 0, 0));
  EXPECT_EQ(w->cfr_.LookUpLayerId("2d"), 0);
  EXPECT_EQ(w->cfr_.GetCategoryBits(w->layers_[0]->names_), 0b1);

  // check that layer 1 settings are loaded correctly
  EXPECT_STREQ(w->layers_[1]->name_.c_str(), "3d");
  ASSERT_EQ(w->layers_[1]->names_.size(), 3);
  EXPECT_STREQ(w->layers_[1]->names_[0].c_str(), "3d");
  EXPECT_STREQ(w->layers_[1]->names_[1].c_str(), "4d");
  EXPECT_STREQ(w->layers_[1]->names_[2].c_str(), "5d");
  EXPECT_EQ(w->layers_[1]->Type(), Entity::EntityType::LAYER);
  EXPECT_TRUE(BodyEq(w->layers_[1]->body_, "3d", b2_staticBody, {0.0, 0.0, 0.0},
                     {1, 1, 1, 1}, 0, 0));
  EXPECT_EQ(w->cfr_.LookUpLayerId("3d"), 1);
  EXPECT_EQ(w->cfr_.LookUpLayerId("4d"), 2);
  EXPECT_EQ(w->cfr_.LookUpLayerId("5d"), 3);
  EXPECT_EQ(w->cfr_.GetCategoryBits(w->layers_[1]->names_), 0b1110);

  // check that layer 2 settings are loaded correctly
  EXPECT_STREQ(w->layers_[2]->name_.c_str(), "lines");
  ASSERT_EQ(w->layers_[2]->names_.size(), 1);
  EXPECT_STREQ(w->layers_[2]->names_[0].c_str(), "lines");
  EXPECT_EQ(w->layers_[2]->Type(), Entity::EntityType::LAYER);
  EXPECT_TRUE(BodyEq(w->layers_[2]->body_, "lines", b2_staticBody,
                     {-1.20, -5, 1.23}, {1, 1, 1, 1}, 0, 0));
  EXPECT_EQ(w->cfr_.LookUpLayerId("lines"), 4);

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
      std::pair<b2Vec2, b2Vec2>(b2Vec2(0.25, 0.25), b2Vec2(0.25, 0.00))};

  std::vector<b2EdgeShape> layer0_edges;
  for (b2Fixture *f = w->layers_[0]->body_->physics_body_->GetFixtureList(); f;
       f = f->GetNext()) {
    b2EdgeShape e = *(dynamic_cast<b2EdgeShape *>(f->GetShape()));
    layer0_edges.push_back(e);

    // check that collision groups are correctly assigned
    ASSERT_EQ(f->GetFilterData().categoryBits, 0x1);
    ASSERT_EQ(f->GetFilterData().maskBits, 0x1);
  }
  EXPECT_EQ(layer0_edges.size(), layer0_expected_edges.size());
  EXPECT_TRUE(do_edges_exactly_match(layer0_edges, layer0_expected_edges));

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
    ASSERT_EQ(f->GetFilterData().categoryBits, 0b1110);
    ASSERT_EQ(f->GetFilterData().maskBits, 0b1110);
  }
  EXPECT_EQ(layer1_edges.size(), layer1_expected_edges.size());
  EXPECT_TRUE(do_edges_exactly_match(layer1_edges, layer1_expected_edges));

  // check layer[2] data
  std::vector<std::pair<b2Vec2, b2Vec2>> layer2_expected_edges = {
      std::pair<b2Vec2, b2Vec2>(b2Vec2(0.1, 0.2), b2Vec2(0.3, 0.4)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(-0.1, -0.2), b2Vec2(-0.3, -0.4)),
      std::pair<b2Vec2, b2Vec2>(b2Vec2(0.01, 0.02), b2Vec2(0.03, 0.04))};

  std::vector<b2EdgeShape> layer2_edges;
  for (b2Fixture *f = w->layers_[2]->body_->physics_body_->GetFixtureList(); f;
       f = f->GetNext()) {
    b2EdgeShape e = *(dynamic_cast<b2EdgeShape *>(f->GetShape()));
    layer2_edges.push_back(e);

    // check that collision groups are correctly assigned
    ASSERT_EQ(f->GetFilterData().categoryBits, 0b10000);
    ASSERT_EQ(f->GetFilterData().maskBits, 0b10000);
  }
  EXPECT_EQ(layer2_edges.size(), layer2_expected_edges.size());
  EXPECT_TRUE(do_edges_exactly_match(layer2_edges, layer2_expected_edges));

  // Check loaded model data
  // Check model 0
  Model *m0 = w->models_[0];
  EXPECT_STREQ(m0->name_.c_str(), "turtlebot1");
  EXPECT_STREQ(m0->namespace_.c_str(), "");
  ASSERT_EQ(m0->bodies_.size(), 5);
  ASSERT_EQ(m0->joints_.size(), 4);

  // check model 0 body 0
  EXPECT_TRUE(BodyEq(m0->bodies_[0], "base", b2_dynamicBody, {0, 0, 0},
                     {1, 1, 0, 0.25}, 0.1, 0.125));
  auto fs = GetBodyFixtures(m0->bodies_[0]);
  ASSERT_EQ(fs.size(), 2);
  EXPECT_TRUE(FixtureEq(fs[0], false, 0, 0xFFFF, 0xFFFF, 0, 0, 0));
  EXPECT_TRUE(CircleEq(fs[0], 0, 0, 1.777));
  EXPECT_TRUE(FixtureEq(fs[1], false, 0, 0xFFFF, 0xFFFF, 982.24, 0.59, 0.234));
  EXPECT_TRUE(
      PolygonEq(fs[1], {{-0.1, 0.1}, {-0.1, -0.1}, {0.1, -0.1}, {0.1, 0.1}}));

  // check model 0 body 1
  EXPECT_TRUE(BodyEq(m0->bodies_[1], "left_wheel", b2_dynamicBody, {-1, 0, 0},
                     {1, 0, 0, 0.25}, 0, 0));
  fs = GetBodyFixtures(m0->bodies_[1]);
  ASSERT_EQ(fs.size(), 1);
  EXPECT_TRUE(FixtureEq(fs[0], true, 0, 0b01, 0b01, 0, 0, 0));
  EXPECT_TRUE(PolygonEq(
      fs[0], {{-0.2, 0.75}, {-0.2, -0.75}, {0.2, -0.75}, {0.2, 0.75}}));

  // check model 0 body 2
  EXPECT_TRUE(BodyEq(m0->bodies_[2], "right_wheel", b2_dynamicBody, {1, 0, 0},
                     {0, 1, 0, 0.25}, 0, 0));
  fs = GetBodyFixtures(m0->bodies_[2]);
  ASSERT_EQ(fs.size(), 1);
  EXPECT_TRUE(FixtureEq(fs[0], false, 0, 0xFFFF, 0xFFFF, 0, 0, 0));
  EXPECT_TRUE(PolygonEq(
      fs[0], {{-0.2, 0.75}, {-0.2, -0.75}, {0.2, -0.75}, {0.2, 0.75}}));

  // check model 0 body 3
  EXPECT_TRUE(BodyEq(m0->bodies_[3], "tail", b2_dynamicBody, {0, 0, 0.52},
                     {0, 0, 0, 0.5}, 0, 0));
  fs = GetBodyFixtures(m0->bodies_[3]);
  ASSERT_EQ(fs.size(), 1);
  EXPECT_TRUE(FixtureEq(fs[0], false, 0, 0b10, 0b10, 0, 0, 0));
  EXPECT_TRUE(PolygonEq(fs[0], {{-0.2, 0}, {-0.2, -5}, {0.2, -5}, {0.2, 0}}));

  // check model 0 body 4
  EXPECT_TRUE(BodyEq(m0->bodies_[4], "antenna", b2_dynamicBody, {0, 0.5, 0},
                     {0.2, 0.4, 0.6, 1}, 0, 0));
  fs = GetBodyFixtures(m0->bodies_[4]);
  ASSERT_EQ(fs.size(), 1);
  EXPECT_TRUE(FixtureEq(fs[0], false, 0, 0b0, 0b0, 0, 0, 0));
  EXPECT_TRUE(CircleEq(fs[0], 0.01, 0.02, 0.25));

  // Check loaded joint data
  EXPECT_TRUE(JointEq(m0->joints_[0], "left_wheel_weld", {0.1, 0.2, 0.3, 0.4},
                      m0->bodies_[0], {-1, 0}, m0->bodies_[1], {0, 0}, true));
  EXPECT_TRUE(WeldEq(m0->joints_[0], 1.57079633, 10, 0.5));

  EXPECT_TRUE(JointEq(m0->joints_[1], "right_wheel_weld", {1, 1, 1, 0.5},
                      m0->bodies_[0], {1, 0}, m0->bodies_[2], {0, 0}, false));
  EXPECT_TRUE(WeldEq(m0->joints_[1], 0, 0, 0));

  EXPECT_TRUE(JointEq(m0->joints_[2], "tail_revolute", {1, 1, 1, 0.5},
                      m0->bodies_[0], {0, 0}, m0->bodies_[3], {0, 0}, false));
  EXPECT_TRUE(RevoluteEq(m0->joints_[2], false, {}));

  EXPECT_TRUE(JointEq(m0->joints_[3], "antenna_revolute", {1, 1, 1, 0.5},
                      m0->bodies_[0], {0, 0}, m0->bodies_[4], {0, 0}, true));
  EXPECT_TRUE(RevoluteEq(m0->joints_[3], true, {-0.002, 3.735}));

  // Check model 1 is same yaml file as model 1, simply do a simple sanity check
  Model *m1 = w->models_[1];
  EXPECT_STREQ(m1->name_.c_str(), "turtlebot2");
  EXPECT_STREQ(m1->namespace_.c_str(), "robot2");
  ASSERT_EQ(m1->bodies_.size(), 5);
  ASSERT_EQ(m1->joints_.size(), 4);

  // check the applied transformation just for the first body
  EXPECT_TRUE(BodyEq(m1->bodies_[0], "base", b2_dynamicBody, {3, 4.5, 3.14159},
                     {1, 1, 0, 0.25}, 0.1, 0.125));

  // Check model 2 which is the chair
  Model *m2 = w->models_[2];
  EXPECT_STREQ(m2->name_.c_str(), "chair1");
  ASSERT_EQ(m2->bodies_.size(), 1);
  ASSERT_EQ(m2->joints_.size(), 0);

  // Check model 2 fixtures
  EXPECT_TRUE(BodyEq(m2->bodies_[0], "chair", b2_staticBody, {1.2, 3.5, 2.123},
                     {1, 1, 1, 0.5}, 0, 0));
  fs = GetBodyFixtures(m2->bodies_[0]);
  ASSERT_EQ(fs.size(), 2);
  EXPECT_TRUE(FixtureEq(fs[0], false, 0, 0b1100, 0b1100, 0, 0, 0));
  EXPECT_TRUE(CircleEq(fs[0], 0, 0, 1));

  EXPECT_TRUE(FixtureEq(fs[1], false, 0, 0xFFFF, 0xFFFF, 0, 0, 0));
  EXPECT_TRUE(CircleEq(fs[1], 0, 0, 0.2));

  // Check model 3 which is the chair
  Model *m3 = w->models_[3];
  EXPECT_STREQ(m3->name_.c_str(), "person1");
  ASSERT_EQ(m3->bodies_.size(), 1);
  ASSERT_EQ(m3->joints_.size(), 0);

  // check the body only
  EXPECT_TRUE(BodyEq(m3->bodies_[0], "body", b2_kinematicBody, {0, 1, 2},
                     {0, 0.75, 0.75, 0.25}, 0, 0));
}

/**
 * This test tries to loads a non-existent world yaml file. It should throw
 * an exception
 */
TEST_F(LoadWorldTest, wrong_world_path) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/random_path/world.yaml");
  test_yaml_fail(
      "Flatland YAML: File does not exist, "
      "path=\".*/load_world_tests/random_path/world.yaml\".*");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(LoadWorldTest, world_invalid_A) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_A/world.yaml");
  test_yaml_fail("Flatland YAML: Entry \"properties\" does not exist");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(LoadWorldTest, world_invalid_B) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_B/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry \"color\" must have size of exactly 4 \\(in "
      "\"layers\" index=0\\)");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(LoadWorldTest, world_invalid_C) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_C/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry index=0 contains unrecognized entry\\(s\\) "
      "\\{\"random_param_1\", \"random_param_2\", \"random_param_3\"\\} \\(in "
      "\"layers\"\\)");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(LoadWorldTest, world_invalid_D) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_D/world.yaml");
  test_yaml_fail("Flatland YAML: Layer with name \"layer\" already exists");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(LoadWorldTest, world_invalid_E) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_E/world.yaml");
  test_yaml_fail("Flatland YAML: Model with name \"turtlebot\" already exists");
}

/**
 * This test tries to loads a invalid world yaml file. It should throw
 * an exception.
 */
TEST_F(LoadWorldTest, world_invalid_F) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/world_invalid_F/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Unable to add 3 additional layer\\(s\\) \\{layer_15, "
      "layer_16, layer_17\\}, current layers count is 14, max allowed is 16");
}

/**
 * This test tries to loads valid world yaml file which in turn tries to
 * load a invalid map yaml file. It should throw an exception.
 */
TEST_F(LoadWorldTest, map_invalid_A) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/map_invalid_A/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry \"origin\" does not exist \\(in layer "
      "\"2d\"\\)");
}

/**
 * This test tries to loads valid world yaml file which in turn load a map yaml
 * file which then inturn tries to load a non-exists map image file. It should
 * throw an exception
 */
TEST_F(LoadWorldTest, map_invalid_B) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/map_invalid_B/world.yaml");
  test_yaml_fail("Flatland YAML: Failed to load \".*\\.png\" in layer \"2d\"");
}

/**
 * This test tries to load a invalid map configuration, an exception should be
 * thrown
 */
TEST_F(LoadWorldTest, map_invalid_C) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/map_invalid_C/world.yaml");
  test_yaml_fail(
      "Flatland File: Failed to load \".*/map_invalid_C/random_file.dat\"");
}

/**
 * This test tries to load a invalid map configuration, an exception should be
 * thrown
 */
TEST_F(LoadWorldTest, map_invalid_D) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/map_invalid_D/world.yaml");
  test_yaml_fail(
      "Flatland File: Failed to read line segment from line 6, in file "
      "\"map_lines.dat\"");
}

/**
 * This test tries to load a invalid map configuration, an exception should be
 * thrown
 */
TEST_F(LoadWorldTest, map_invalid_E) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/map_invalid_E/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry \"scale\" does not exist \\(in layer \"lines\"\\)");
}

/**`
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_A) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_A/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry \"bodies\" must be a list \\(in model "
      "\"turtlebot\"\\)");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_B) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_B/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry \"points\" must have size >= 3 \\(in model "
      "\"turtlebot\" body \"base\" \"footprints\" index=1\\)");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_C) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_C/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry \"anchor\" must have size of exactly 2 \\(in model "
      "\"turtlebot\" joint \"right_wheel_weld\" \"bodies\" index=1\\)");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_D) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_D/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Cannot find body with name \"left_wheel_123\" in model "
      "\"turtlebot\" joint \"left_wheel_weld\" \"bodies\" index=1");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_E) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_E/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Invalid footprint \"layers\" in model \"turtlebot\" body "
      "\"left_wheel\" \"footprints\" index=0, \\{random_layer\\} layer\\(s\\) "
      "does not exist");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_F) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_F/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry index=0 contains unrecognized entry\\(s\\) "
      "\\{\"random_paramter\"\\} \\(in model \"turtlebot\" body \"base\" "
      "\"footprints\"\\)");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_G) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_G/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry body \"base\" contains unrecognized entry\\(s\\) "
      "\\{\"random_paramter\"\\} \\(in model \"turtlebot\"\\)");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_H) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_H/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Invalid \"bodies\" in \"turtlebot\" model, body with "
      "name \"base\" already exists");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_I) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_I/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Invalid \"joints\" in \"turtlebot\" model, joint with "
      "name \"wheel_weld\" already exists");
}

/**
 * This test tries to load a invalid model yaml file, it should fail
 */
TEST_F(LoadWorldTest, model_invalid_J) {
  world_yaml =
      this_file_dir / fs::path("load_world_tests/model_invalid_J/world.yaml");
  test_yaml_fail(
      "Flatland YAML: Entry \"points\" must have size <= 8 \\(in model "
      "\"turtlebot\" body \"base\" \"footprints\" index=1\\)");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  ros::init(argc, argv, "load_world_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
