/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 collision_filter_registry.cpp
 * @brief	 Implements Collision Filter Registrar
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

#include <flatland_server/collision_filter_registry.h>

namespace flatland_server {

const int CollisionFilterRegistry::LAYER_NOT_EXIST;
const int CollisionFilterRegistry::LAYER_ALREADY_EXIST;
const int CollisionFilterRegistry::LAYERS_FULL;
const int CollisionFilterRegistry::MAX_LAYERS;

CollisionFilterRegistry::CollisionFilterRegistry()
    : no_collide_group_cnt_(0), collide_group_cnt_(0) {}

int CollisionFilterRegistry::RegisterCollide() {
  collide_group_cnt_++;
  return collide_group_cnt_;
}

int CollisionFilterRegistry::RegisterNoCollide() {
  no_collide_group_cnt_--;
  return no_collide_group_cnt_;
}

bool CollisionFilterRegistry::IsLayersFull() const {
  return layer_id_table_.size() >= MAX_LAYERS;
}

int CollisionFilterRegistry::RegisterLayer(std::string layer_name) {
  if (IsLayersFull()) {
    return LAYERS_FULL;
  }

  if (layer_id_table_.count(layer_name) > 0) {
    return LAYER_ALREADY_EXIST;
  }

  // You have maximum number of ID you can assign. Loop through all the ones
  // assigned currently and find on that is not used
  int i;
  for (i = 0; i < MAX_LAYERS; i++) {
    std::map<std::string, int>::iterator it;
    for (it = layer_id_table_.begin(); it != layer_id_table_.end(); it++) {
      if (it->second == i) {
        break;
      }
    }

    if (it == layer_id_table_.end()) {
      layer_id_table_[layer_name] = i;
      break;
    }
  }

  return i;
}

int CollisionFilterRegistry::LookUpLayerId(std::string layer_name) const {
  if (layer_id_table_.count(layer_name) == 0) {
    return LAYER_NOT_EXIST;
  }
  return layer_id_table_.at(layer_name);
}

std::vector<std::string> CollisionFilterRegistry::GetAllLayers() const {
  std::vector<std::string> layer_names;

  std::map<std::string, int>::const_iterator it;
  for (it = layer_id_table_.begin(); it != layer_id_table_.end(); it++) {
    layer_names.push_back(it->first);
  }

  return layer_names;
}

int CollisionFilterRegistry::LayersCount() const {
  return layer_id_table_.size();
}

uint16_t CollisionFilterRegistry::GetCategoryBits(
    const std::vector<std::string> &layers,
    std::vector<std::string> *invalid_layers) const {
  if (layers.size() == 1 && layers[0] == "all") {
    return ~((uint16_t)0x0);
  }

  if (invalid_layers) {
    invalid_layers->clear();
  }
  uint16_t category_bits = 0;

  for (const auto &layer : layers) {
    int layer_id = LookUpLayerId(layer);

    if (layer_id < 0 && invalid_layers) {
      invalid_layers->push_back(layer);
    } else {
      category_bits |= 1 << layer_id;
    }
  }

  return category_bits;
}

};  // namespace flatland_server
