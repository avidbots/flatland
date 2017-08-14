/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 collision_filter_registry.h
 * @brief	 Defines Collision Filter Registrar
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

#ifndef FLATLAND_SERVER_COLLISION_FILTER_REGISTRY_H
#define FLATLAND_SERVER_COLLISION_FILTER_REGISTRY_H

#include <map>
#include <string>
#include <vector>

namespace flatland_server {

/**
 * This class defines a collision filter registry. It allows layers to register
 * for unique ID's, which are used for setting Box2D collision categories
 * and mask bits. The footprints in the model will use this to put itself on
 * the correct collision category. It also hands out unique collide (positive
 * numbers) and no collide (negative numbers) Box2D collision groups.
 */
class CollisionFilterRegistry {
 public:
  static const int LAYER_NOT_EXIST = -1;      ///< No such layer
  static const int LAYER_ALREADY_EXIST = -2;  ///< Layer exists
  static const int LAYERS_FULL = -3;          ///< Cannot add more layers
  static const int MAX_LAYERS = 16;  ///< 16 is the maximum as defined by Box2D

  /// internal counter to keep track of no collides groups
  int no_collide_group_cnt_;
  /// internal counter to keep track of collide groups
  int collide_group_cnt_;
  std::map<std::string, int> layer_id_table_;  ///< Layer name to ID LUT

  /**
   * @brief Constructor for the collision filter registry
   */
  CollisionFilterRegistry();

  /**
   * @brief Get a new and unique collision group, +ve numbers
   */
  int RegisterCollide();

  /**
   * @brief Get a new and unique no collision group, -ve numbers
   */
  int RegisterNoCollide();

  /**
   * @brief Check if the number of layers maxed out
   * @return if layers are full
   */
  bool IsLayersFull() const;

  /**
   * @brief Register a new layer
   * @param[in] layer Name of the layer
   * @return assigned ID for the registered layer, or error codes LAYERS_FULL or
   * LAYER_ALREADY_EXIST
   */
  int RegisterLayer(std::string layer);

  /**
   * @brief get layer ID
   * @param[in] name Name of the layer
   * @return the id of the layer, or LAYER_NOT_EXIST
   */
  int LookUpLayerId(std::string name) const;

  /**
   * @brief Get all registered layers
   * @return vector to store layer names in, will be cleared
   */
  std::vector<std::string> GetAllLayers() const;

  /**
   * @brief Get number of layers
   * @return number of layers
   */
  int LayersCount() const;

  /**
   * @brief: Get the Box2D category bits from a list of layers
   * @param[in] layers The layers for generating the category bits, if the input
   * exactly equals to {"all"}, it returns all bits to 1 (0xFFFF)
   * @param[out] invalid_layers if a given layer does not exist, it is pushed to
   * this list, optional
   */
  uint16_t GetCategoryBits(
      const std::vector<std::string> &layers,
      std::vector<std::string> *invalid_layers = nullptr) const;
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_COLLISION_FILTER_REGISTRY_H
