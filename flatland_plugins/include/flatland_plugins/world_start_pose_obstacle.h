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
 * @brief   Interface for StartPoseObstacle Plugin
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
#include <ros/ros.h>
#include <Box2D/Box2D.h>
#include <string>
#include <map>

#ifndef FLATLAND_PLUGINS_WORLD_START_POSE_OBSTACLE_H
#define FLATLAND_PLUGINS_WORLD_START_POSE_OBSTACLE_H

using namespace flatland_server;

namespace flatland_plugins {
  // sub class for loading laser Param
  struct LaserRead {
    std::string name_; // name of the laser
    b2Vec2 location_;  // location of the laser in local frame
    double range_;     // range of the laser
    double min_angle_;
    double max_angle_;
    double increment_;
    LaserRead(std::string name, b2Vec2 location, double range, double min_angle, double max_angle, double increment):
    name_(name), location_(location), range_(range), min_angle_(min_angle), max_angle_(max_angle), increment_(increment){}
  };

  class StartPoseObstacle : public WorldPlugin {
    double start_pose_ratio_;
    double start_pose_range_;
    std::vector<LaserRead>laser_list_;
    std::map<b2EdgeShape *, int>Wall_Laser_Hit_Map_; // map for each wall and how many laser hits is on them
    void OnInitialize(const YAML::Node &config) override;
    void ReadLasers(std::string yaml_path);
  }
}

#endif //FLATLAND_PLUGINS_WORLD_START_POSE_OBSTACLE_H