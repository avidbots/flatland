/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  world_start_pose_obstacle.h
 * @brief   Add obstacles around the start pose of the robot
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

#include <flatland_server/world_plugin.h>
#include <flatland_server/types.h>
#include <flatland_plugins/world_modifier.h>
#include <flatland_plugins/world_start_pose_obstacle.h>
#include <ros/ros.h>
#include <Box2D/Box2D.h>
#include <string>

using namespace flatland_server;

namespace flatland_plugins {


  StartPoseObstacle::OnInitialize(const YAML::Node &config) {
    // read in the plugin config 
    YamlReader plugin_reader(config);
    std::string layer_name = plugin_reader.Get<std::string>("layer", "");
    start_pose_ratio_ = plugin_reader.Get<double>("start_pose_ratio", 0);
    start_pose_range_ = plugin_reader.Get<double>("start_pose_range", 0);
    double wall_wall_dist = plugin_reader.Get<double>("wall_wall_dist", 1);
    bool double_wall = plugin_reader.Get<bool>("double_wall", false);
    std::string robot_name = plugin_reader.Get<std::string>("robot_name", "");
    Layer * layer = NULL;
    for(auto & it : world_->layers_) {
      for(auto & v_it : it.first) {
        if(v_it == layer_name) {
          layer = it.second;
          break;
        }
      }
    }
    if(layer == NULL) {
     throw("no such layer name!");
    }
    // read in the robot location from the world.yaml
    Pose robot_ini_pose;
    YamlReader models_reader = world_config_.SubnodeOpt("models", YamlReader::LIST);
    if(models_reader.IsNodeNull()) {
      throw("no robot specified!");
    } 
    for(int i = 0; i < models_reader.NodeSize(); i++) {
      YamlReader reader = models_reader.Subnode(i, YamlReader::MAP);
      if(i+1 >= models_reader.NodeSize() && reader.Get<std::string>("name") != robot_name) {
        throw("cannot find specified robot name of " + robot_name);
      }
      if(reader.Get<std::string>("name") == robot_name) {
        robot_ini_pose = reader.Get("pose", Pose(0,0,0));
        b2Transform tran = layer->body_->physics_body_->GetTransform();
        b2Vec2 ini_pose = b2MulT(tran_, b2Vec2(robot_ini_pose.x, robot_ini_pose.y));
        robot_ini_pose.x = ini_pose.x;
        robot_ini_pose.y = ini_pose.y;

        // read all the lasers
        std::string model_yaml_path = reader.Get<std::string>("model");
        boost::filesystem::path model_path(model_yaml_path);
        if (model_yaml_path.front() != '/') {
          model_path = world_yaml_dir / model_path;
        }
        YamlReader model_reader = YamlReader(model_path.string());
        YamlReader plugin_reader = model_reader.SubnodeOpt("plugins", YamlReader::LIST);
        if(plugin_reader.IsNodeNull()) {
          throw("no laser on the robot!");
        }

        int laser_total_count = 0; // total number of laser points
        for(int j = 0; j < plugin_reader.NodeSize(); j++) {
          YamlReader laser_reader = plugin_reader.Subnode(j, YamlReader::MAP);
          if(laser_reader.Get<std::string>("type") == "Laser") {
            laser_total_count += std::lround((laser_reader.Subnode("angle", YamlReader::MAP).Get<double>("max",0)
              - laser_reader.Subnode("angle", YamlReader::MAP).Get<double>("min",0))
            / laser_reader.Subnode("angle", YamlReader::MAP).Get<double>("increment",0)) + 1;

            // transform laser's position from robot frame to local frame
            Pose laser_pose = laser_reader.Get("pose", Pose(0,0,0));
            cout << "laser_pose is " << laser_pose.x << ", " << laser_pose.y << endl;
            b2Transform robot_transform(b2Vec2(robot_ini_pose.x,robot_ini_pose.y), b2Rot(robot_ini_pose.theta));
            b2Vec2 lase_pose_transformed = b2Mul(robot_transform, b2Vec2(laser_pose.x, laser_pose.y));
            laser_pose.x = lase_pose_transformed.x;
            laser_pose.y = lase_pose_transformed.y;

            // create laser_read object
            laser_list_.push_back(LaserRead(laser_reader.Get<std::string>("name",""), 
              b2Vec2(laser_pose.x, laser_pose.y), 
              laser_reader.Get<double>("range",0), 
              laser_reader.Subnode("angle", YamlReader::MAP).Get<double>("min",0) + laser_pose.theta,
              laser_reader.Subnode("angle", YamlReader::MAP).Get<double>("max",0) + laser_pose.theta, 
            laser_reader.Subnode("angle", YamlReader::MAP).Get<double>("increment",0)));  
          } 
        }
        // compute how many laser hits we want to modify
        start_pose_wall_num_ = std::lround(laser_total_count * start_pose_ratio_);
      }
    }
    // create the world modifiyer
    WorldModifier modifier(world_, layer_name, wall_wall_dist, double_wall, robot_ini_pose);

    if(start_pose_ratio_) {
      // first initial wall_laser_hit_map
      for (auto const & x : Wall_List) { 
        Wall_Laser_Hit_Map_[x] = 0; 
      }
            
      for(auto & laser_it : laser_list_) { // loop through all lasers
        LaserRayCast(laser_it, start_pose_range_);
      }
      
      // dump the map int a vector to sort by value
      std::vector<std::pair<b2EdgeShape *, int>>Wall_laser_hit_list;
      for(auto &wall_laser_hit_it : Wall_Laser_Hit_Map_) {
        Wall_laser_hit_list.push_back(wall_laser_hit_it);
      } 
      sort(Wall_laser_hit_list.begin(), Wall_laser_hit_list.end(), 
        [=](std::pair<b2EdgeShape *, int> &a, std::pair<b2EdgeShape *, int> &b) {
          return a.second > b.second;
        }
      );
      int modification_count = 0;
      for(int j = 0; j < (int)Wall_laser_hit_list.size(); j++) {
        if(modification_count >= start_pose_wall_num_) {
          break;
        }
        cout << "adding start_pose wall with acc of " << Wall_laser_hit_list[j].second << endl;
        modification_count += Wall_laser_hit_list[j].second;
        AddFullWall(Wall_laser_hit_list[j].first);
      }
    }
  }


}