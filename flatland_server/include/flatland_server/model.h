/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 model.h
 * @brief	 Defines flatland Model
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

#ifndef FLATLAND_SERVER_MODEL_H
#define FLATLAND_SERVER_MODEL_H

#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/entity.h>
#include <flatland_server/joint.h>
#include <flatland_server/model_body.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

namespace flatland_server {

class ModelBody;
class Joint;

/**
 * This class defines a Model. It can be used to repsent any object in the
 * environment such robots, chairs, deskes etc.
 */
class Model : public Entity {
 public:
  std::string namespace_;            ///< namespace of the model
  std::vector<ModelBody *> bodies_;  ///< list of bodies in the model
  std::vector<Joint *> joints_;      ///< list of joints in the model
  YamlReader plugins_reader_;        ///< for storing plugins when paring YAML
  CollisionFilterRegistry *cfr_;     ///< Collision filter registry
  std::string viz_name_;             ///< used for visualization

  /**
   * @brief Constructor for the model
   * @param[in] physics_world Box2D physics world
   * @param[in] cfr Collision filter registry
   * @param[in] name Name of the model
   */
  Model(b2World *physics_world, CollisionFilterRegistry *cfr,
        const std::string &ns, const std::string &name);

  /**
   * @brief Destructor for the layer class
   */
  ~Model();

  /**
   * @brief Return the type of entity
   * @return Model type
   */
  EntityType Type() const { return EntityType::MODEL; }

  /**
   * @brief load bodies to this model, throws exceptions upon failure
   * @param[in] bodies_reader YAML reader for node containing the list of bodies
   */
  void LoadBodies(YamlReader &bodies_reader);

  /**
   * @brief load joints to this model, throws exceptions upon failure
   * @param[in] joints_reader YAML reader for node containing the list of joints
   */
  void LoadJoints(YamlReader &joints_reader);

  /**
   * @brief Get a body in the model using its name
   * @param[in] name Name of the body
   * @return pointer to the body, nullptr indicates body cannot be found
   */
  ModelBody *GetBody(const std::string &name);

  /**
   * @brief Get a body in the model using its name
   * @param[in] name Name of the joint
   * @return pointer to the joint, nullptr if the joint does not exist
   */
  Joint *GetJoint(const std::string &name);

  /**
   * @return List of bodies the model has
   */
  const std::vector<ModelBody *> &GetBodies();

  /**
   * @return List of joints the model has
   */
  const std::vector<Joint *> &GetJoints();

  /**
   * @return The namespace of the model
   */
  const std::string &GetNameSpace() const;

  /**
   * @return prepend the tf with namespace_
   */
  std::string NameSpaceTF(const std::string &frame_id) const;

  /**
   * @return prepend the topic with namespace/
   */
  std::string NameSpaceTopic(const std::string &topic) const;

  /**
   * @return The name of the model
   */
  const std::string &GetName() const;

  /**
   * @return The collision filter registrar
   */
  const CollisionFilterRegistry *GetCfr() const;

  /**
   * @brief Publish debug visualizations for model
   */
  void DebugVisualize() const override;

  /**
   * @brief log debug messages for the layer
   */
  void DebugOutput() const override;

  /**
   * @brief Dump box2d data for debugging
   */
  void DumpBox2D() const;

  /**
   * @brief transform all bodies in the model
   * @param[in] pose_delta dx, dy, dyaw
   */
  void TransformAll(const Pose &pose_delta);

  /**
   * @brief Explicitly set the model pose in world coordinates
   * @param[in] pose world x, world y, world yaw
   */
  void SetPose(const Pose &pose);

  /**
   * @brief Create a model, throws exceptions upon failure
   * @param[in] physics_world Box2D physics world
   * @param[in] cfr Collision filter registry
   * @param[in] model_yaml_path Absolute path to the model yaml file
   * @param[in] ns Namespace of the robot
   * @param[in] name Name of the model
   * @return A new model
   */
  static Model *MakeModel(b2World *physics_world, CollisionFilterRegistry *cfr,
                          const std::string &model_yaml_path,
                          const std::string &ns, const std::string &name);
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_MODEL_H
