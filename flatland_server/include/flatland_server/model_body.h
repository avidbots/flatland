/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 model_body.h
 * @brief	 Defines Model Body
 * @author   Chunshang Li
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

#ifndef FLATLAND_SERVER_MODEL_BODY_H
#define FLATLAND_SERVER_MODEL_BODY_H

#include <flatland_server/body.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/model.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

class Model;

/**
 * This class defines a model body which is a body that is always used together
 * with a model. It contains members and useful methods that is specfic for a
 * body in a model
 */
class ModelBody : public Body {
 public:
  CollisionFilterRegistry *cfr_;  ///< collision filter registry

  /**
   * @brief Constructor for the Model Body
   * @param[in] physics_world Box2D physics world
   * @param[in] cfr Collision filter registry
   * @param[in] model Model the body belongs to
   * @param[in] name Name of the body
   * @param[in] color Color of the body for visualization
   * @param[in] pose The pose to place the body at
   * @param[in] body_type Type of Box2D body, either dynamic, static, or
   * kinematic
   * @param[in] properties per-object YAML properties for plugins to reference
   * @param[in] linear_damping Box2D body linear damping
   * @param[in] angular_damping Box2D body angular damping
   */
  ModelBody(b2World *physics_world, CollisionFilterRegistry *cfr, Model *model,
            const std::string &name, const Color &color, const Pose &pose,
            b2BodyType body_type, const YAML::Node &properties,
            double linear_damping, double angular_damping);

  /**
   * @return The collision filter registry
   */
  const CollisionFilterRegistry *GetCfr() const;

  /**
   * @brief Load footprints (Box2D fixtures) into the body
   * @param[in] footprints_reader YAML reader for node containing the footprints
   * parameters
   */
  void LoadFootprints(YamlReader &footprints_reader);

  /**
   * @brief Configures the common properties of footprints
   * @param[in] footprint_reader YAML reader for node containing the footprint
   * parameters
   * @param[out] fixture_def Box2D fixture definition
   */
  void ConfigFootprintDef(YamlReader &footprint_reader,
                          b2FixtureDef &fixture_def);

  /**
   * @brief Loads a circle footprint
   * @param[in] footprint_reader YAML reader for node containing the footprint
   * parameters
   */
  void LoadCircleFootprint(YamlReader &footprint_reader);

  /**
   * @brief Loads a circle footprint
   * @param[in] footprint_reader YAML reader for node containing the footprint
   * parameters
   */
  void LoadPolygonFootprint(YamlReader &footprint_reader);

  /**
   * @brief Factory method to create a model body
   * @param[in] physics_world Box2D physics world
   * @param[in] cfr Collision filter registry
   * @param[in] model The model this model body belongs to
   * @param[in] body_node YAML reader for node containing the body parameters
   */
  static ModelBody *MakeBody(b2World *physics_world,
                             CollisionFilterRegistry *cfr, Model *model,
                             YamlReader &body_node);
};
};      // namespace flatland_server
#endif  // FLATLAND_MODEL_BODY_H
