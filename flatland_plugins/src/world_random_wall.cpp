/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  world_random_wall.h
 * @brief   a simple plugin that add random walls on the field
 * @author  Yi Ren
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
#include <flatland_plugins/world_modifier.h>
#include <flatland_plugins/world_random_wall.h>
#include <flatland_server/types.h>
#include <flatland_server/world_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <iostream>
#include <string>

using namespace flatland_server;
using std::cout;
using std::endl;
namespace flatland_plugins {
void RandomWall::OnInitialize(const YAML::Node &config) {
  // read in the plugin config
  YamlReader plugin_reader(config);
  std::string layer_name = plugin_reader.Get<std::string>("layer", "");
  unsigned int num_of_walls =
      plugin_reader.Get<unsigned int>("num_of_walls", 0);
  double wall_wall_dist = plugin_reader.Get<double>("wall_wall_dist", 1);
  bool double_wall = plugin_reader.Get<bool>("double_wall", false);
  std::string robot_name = plugin_reader.Get<std::string>("robot_name", "");
  Layer *layer = NULL;
  for (auto &it : world_->layers_name_map_) {
    for (auto &v_it : it.first) {
      if (v_it == layer_name) {
        layer = it.second;
        break;
      }
    }
  }
  if (layer == NULL) {
    throw("no such layer name!");
  }

  // read in the robot location from the world.yaml
  Pose robot_ini_pose;
  YamlReader models_reader =
      world_config_.SubnodeOpt("models", YamlReader::LIST);
  if (models_reader.IsNodeNull()) {
    throw("no robot specified!");
  }
  for (int i = 0; i < models_reader.NodeSize(); i++) {
    YamlReader reader = models_reader.Subnode(i, YamlReader::MAP);
    if (i + 1 >= models_reader.NodeSize() &&
        reader.Get<std::string>("name") != robot_name) {
      throw("cannot find specified robot name of " + robot_name);
    }
    if (reader.Get<std::string>("name") == robot_name) {
      robot_ini_pose = reader.Get("pose", Pose(0, 0, 0));
      b2Transform tran = layer->body_->physics_body_->GetTransform();
      b2Vec2 ini_pose =
          b2MulT(tran, b2Vec2(robot_ini_pose.x, robot_ini_pose.y));
      robot_ini_pose.x = ini_pose.x;
      robot_ini_pose.y = ini_pose.y;
      break;
    }
  }
  // create the world modifiyer
  cout << "robot location read" << endl;
  WorldModifier modifier(world_, layer_name, wall_wall_dist, double_wall,
                         robot_ini_pose);

  // get all walls
  std::vector<b2EdgeShape *> Wall_List;
  for (b2Fixture *f = layer->body_->physics_body_->GetFixtureList(); f;
       f = f->GetNext()) {
    Wall_List.push_back(static_cast<b2EdgeShape *>(f->GetShape()));
  }
  std::srand(std::time(0));
  std::random_shuffle(Wall_List.begin(), Wall_List.end());
  try {
    for (unsigned int i = 0; i < num_of_walls; i++) {
      modifier.AddFullWall(Wall_List[i]);
    }
  } catch (std::string e) {
    throw e;
  }
}
};  // namespace

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RandomWall,
                       flatland_server::WorldPlugin)