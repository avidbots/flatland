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
 * @brief Provide general functions to modify the world
 * @author Yi Ren
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
#ifndef WORLD_MODIFIER_H
#define WORLD_MODIFIER_H

#include <Box2D/Box2D.h>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <map>
#include <vector>
#include <string>
#include <flatland_server/yaml_reader.h>
#include <flatland_server/types.h>
#include <flatland_server/world.h>
using namespace flatland_server;



namespace flatland_plugins {
  // forward declaration  
  // class flatland_server::World;  
  struct WorldModifier {
    // private members
    World *world_;        // the world we are modifying
    std::string layer_name_;
    double wall_wall_dist_;
    bool double_wall_;
    Pose robot_ini_pose_;

    /*
    * @brief add wall depend on different orientation of the original wall and the robot
    * @param[in] double d, calculated to determine which side of the old wall will the new be at
    * @param[in] b2Vec2 vertex1, old wall's vertex1
    * @param[in] b2Vec2 vertex2, old wall's vertex2
    * @param[out] b2EdgeShape new_wall, reference passed in to set the vertexs
    */
    void GetWallDirection(double d, b2Vec2 vertex1, b2Vec2 vertex2, b2EdgeShape &new_wall);

    /*
    * @brief add the new wall into the world
    * @param[in] new_wall, the wall that's going to be added 
    */
    void AddWall(b2EdgeShape &new_wall);

    /*
    * @brief add two side walls to make it a full obstacle
    * @param[in] old_wall, the old wall where new wall is added on top to
    * @param[in] new_wall, the new wall got added
    */
    void AddSideWall(b2EdgeShape &old_wall, b2EdgeShape &new_wall);

    /*
     * @brief constructor for WorldModifier
     * @param[in] world, the world that we are adding walls to
     * @param[in] layer_name, which layer is the obstacle going to be added
     * @param[in] wall_wall_dist, how thick is the obstacle
     * @param[in] double_wall, whether add obstacle on both side or not
     * @param[in] robot_ini_pose, the initial pose of the robot
    */
    WorldModifier(flatland_server::World *world, std::string layer_name, double wall_wall_dist, 
      bool double_wall, Pose robot_ini_pose);

    /*
    * @brief make a new wall in front of the old wall, also add two side walls to make a full object
    * @param[in] b2EdgeShape *wall, old wall where new wall will be added on top to
    */
    void AddFullWall(b2EdgeShape *wall);

  }; // class WorldModifier
}; // namespace flatland_server
#endif // WORLD_MODIFIER_H