/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.h
 * @brief	 Loads world file
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

#ifndef FLATLAND_SERVER_WORLD_H
#define FLATLAND_SERVER_WORLD_H

#include <Box2D/Box2D.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <string>
#include <vector>

namespace flatland_server {

/**
 * This class defines a world in the simulation. A world contains layers
 * that can represent environments at multiple levels, and models which are
 * can be robots or obstacles. 
 */
class World {
 public:
  b2World *physics_world_;       ///< Box2D physics world
  b2Vec2 gravity_;               ///< Box2D world gravity, always (0, 0)
  std::vector<Layer *> layers_;  ///< list of layers
  std::vector<Model *> models_;  ///< list of models
  CollisionFilterRegistry cfr_;  ///< collision registry for layers and models

  /**
   * @brief Constructor for the world class. All data required for
   * initialization should be passed in here
   */
  World();

  /**
   * @brief Destructor for the world class
   */
  ~World();

  /**
   * @brief load layers into the world. Throws derivatives of YAML::Exception
   * @param[in] yaml_path Path to the world yaml file containing list of layers
   */
  void LoadLayers(const std::string &yaml_path);

  /**
   * @brief load models into the world. Throws derivatives of YAML::Exception
   * @param[in] yaml_path Path to the world yaml file containing list of models
   */
  void LoadModels(const std::string &yaml_path);

  /**
   * brief @load models into the world. Throws derivatives of YAML::Exception
   * @param[in] model_yaml_path Path to the model yaml file
   * @param[in] name Name of the model
   * @param[in] pose Initial pose of the model in x, y, yaw
   */
  void LoadModel(const std::string &model_yaml_path, const std::string &name,
                 const std::array<double, 3> pose);

  /**
   * @brief factory method to create a instance of the world class. Cleans all
   * the inputs before instantiation of the class. Throws derivatives of
   * YAML::Exception
   * @param[in] yaml_path Path to the world yaml file
   * @return pointer to a new world
   */
  static World *MakeWorld(const std::string &yaml_path);

  /**
   * @brief Publish debug visualizations for everything
   */
  void DebugVisualize();
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_WORLD_H
