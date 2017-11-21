/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  world_modifier.h
 * @brief defintions for functions from world_modifier.h
 * @author Arthur Ren
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
#include <ros/ros.h>

#include <flatland_plugins/world_modifier.h>
#include <flatland_server/layer.h>
#include <flatland_server/types.h>
#include <flatland_server/world.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using std::cout;
using std::endl;
using namespace std::placeholders;
using namespace flatland_server;

namespace flatland_plugins {

float RayTrace::ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                              const b2Vec2 &normal, float fraction) {
  // only register hit in the specified layers
  if (!(fixture->GetFilterData().categoryBits & category_bits_)) {
    // cout << "hit others " << endl;
    return -1.0f;  // return -1 to ignore this hit
  }
  is_hit_ = true;
  fraction_ = fraction;
  return fraction;
}

WorldModifier::WorldModifier(flatland_server::World *world,
                             std::string layer_name, double wall_wall_dist,
                             bool double_wall, Pose robot_ini_pose)
    : world_(world),
      layer_name_(layer_name),
      wall_wall_dist_(wall_wall_dist),
      double_wall_(double_wall),
      robot_ini_pose_(robot_ini_pose) {}

void WorldModifier::CalculateNewWall(double d, b2Vec2 vertex1, b2Vec2 vertex2,
                                     b2EdgeShape &new_wall) {
  b2Vec2 new_wall_v1;
  b2Vec2 new_wall_v2;
  if (d == 0) {  // if distance towards the robot is 0
    ROS_FATAL_NAMED("Node", "robot start pose hit the wall!");
  } else if (d < 0) {              // if on the left side
    if (vertex1.x == vertex2.x) {  // if it is a vertical wall
      new_wall_v1.Set(vertex1.x + wall_wall_dist_, vertex1.y);
      new_wall_v2.Set(vertex2.x + wall_wall_dist_, vertex2.y);
    } else if (vertex1.y == vertex2.y) {  // if it is a horizontal wall
      new_wall_v1.Set(vertex1.x, vertex1.y + wall_wall_dist_);
      new_wall_v2.Set(vertex2.x, vertex2.y + wall_wall_dist_);
    } else {  // if it's an angled wall
      new_wall_v1.Set(vertex1.x + wall_wall_dist_, vertex1.y + wall_wall_dist_);
      new_wall_v2.Set(vertex2.x + wall_wall_dist_, vertex2.y + wall_wall_dist_);
    }
  } else {                         // if on the right side
    if (vertex1.x == vertex2.x) {  // if it is a vertical wall
      new_wall_v1.Set(vertex1.x - wall_wall_dist_, vertex1.y);
      new_wall_v2.Set(vertex2.x - wall_wall_dist_, vertex2.y);
    } else if (vertex1.y == vertex2.y) {  // if it is a horizontal wall
      new_wall_v1.Set(vertex1.x, vertex1.y - wall_wall_dist_);
      new_wall_v2.Set(vertex2.x, vertex2.y - wall_wall_dist_);
    } else {  // if it's an angled wall
      new_wall_v1.Set(vertex1.x - wall_wall_dist_, vertex1.y - wall_wall_dist_);
      new_wall_v2.Set(vertex2.x - wall_wall_dist_, vertex2.y - wall_wall_dist_);
    }
  }
  new_wall.Set(new_wall_v1, new_wall_v2);
}

void WorldModifier::AddWall(b2EdgeShape &new_wall) {
  Layer *layer = NULL;
  std::vector<std::string> cfr_names;
  for (auto &it : world_->layers_name_map_) {
    for (auto &v_it : it.first) {
      if (v_it == layer_name_) {
        layer = it.second;
        cfr_names = it.first;
        break;
      }
    }
  }
  if (layer == NULL) {
    throw("no such layer name!");
  }
  b2FixtureDef fixture_def;
  fixture_def.shape = &new_wall;
  uint16_t categoryBits = layer->cfr_->GetCategoryBits(cfr_names);
  fixture_def.filter.categoryBits = categoryBits;
  fixture_def.filter.maskBits = categoryBits;

  layer->body_->physics_body_->CreateFixture(&fixture_def);
}

void WorldModifier::AddSideWall(b2EdgeShape &old_wall, b2EdgeShape &new_wall) {
  b2Vec2 old_wall_v1 = old_wall.m_vertex1;
  b2Vec2 old_wall_v2 = old_wall.m_vertex2;
  b2Vec2 new_wall_v1 = new_wall.m_vertex1;
  b2Vec2 new_wall_v2 = new_wall.m_vertex2;
  // first side
  double k =
      ((old_wall_v2.y - old_wall_v1.y) * (new_wall_v1.x - old_wall_v1.x) -
       (old_wall_v2.x - old_wall_v1.x) * (new_wall_v1.y - old_wall_v1.y)) /
      (std::pow((old_wall_v2.y - old_wall_v1.y), 2) +
       std::pow((old_wall_v2.x - old_wall_v1.x), 2));
  double x = new_wall_v1.x - k * (old_wall_v2.y - old_wall_v1.y);
  double y = new_wall_v1.y + k * (old_wall_v2.x - old_wall_v1.x);
  b2EdgeShape first_wall;
  first_wall.Set(new_wall_v1, b2Vec2(x, y));
  AddWall(first_wall);

  // second side
  k = ((old_wall_v2.y - old_wall_v1.y) * (new_wall_v2.x - old_wall_v1.x) -
       (old_wall_v2.x - old_wall_v1.x) * (new_wall_v2.y - old_wall_v1.y)) /
      (std::pow((old_wall_v2.y - old_wall_v1.y), 2) +
       std::pow((old_wall_v2.x - old_wall_v1.x), 2));
  x = new_wall_v2.x - k * (old_wall_v2.y - old_wall_v1.y);
  y = new_wall_v2.y + k * (old_wall_v2.x - old_wall_v1.x);
  b2EdgeShape second_wall;
  second_wall.Set(new_wall_v2, b2Vec2(x, y));
  AddWall(second_wall);
}

void WorldModifier::AddFullWall(b2EdgeShape *wall) {
  b2Vec2 vertex1 = wall->m_vertex1;
  b2Vec2 vertex2 = wall->m_vertex2;
  double d = (robot_ini_pose_.x - vertex1.x) * (vertex2.y - vertex1.y) -
             (robot_ini_pose_.y - vertex1.y) * (vertex2.x - vertex1.x);

  // add the main wall
  b2EdgeShape new_wall;
  CalculateNewWall(d, vertex1, vertex2, new_wall);
  AddWall(new_wall);
  // add the sidewall
  AddSideWall(*wall, new_wall);

  if (double_wall_) {  // if add walls on both side
    CalculateNewWall(-d, vertex1, vertex2, new_wall);
    AddWall(new_wall);
    // add the sidewall
    AddSideWall(*wall, new_wall);
  }
}
};  // namespace