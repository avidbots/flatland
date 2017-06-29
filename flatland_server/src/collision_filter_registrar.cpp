/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 collision_filter_registrar.cpp
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

#include <flatland_server/collision_filter_registrar.h>

namespace flatland_server {


CollisionFilterRegistrar::CollisionFilterRegistrar()
  : no_collide_group_cnt_(0), collide_group_cnt_(0){}

int CollisionFilterRegistrar::RegisterCollide() {
  return 0;
}

int CollisionFilterRegistrar::RegisterNoCollide() {
  return 0;
}

bool CollisionFilterRegistrar::IsLayersFull() {
  return layer_id_table_.size() >= MAX_LAYERS;
}

int CollisionFilterRegistrar::RegisterLayer(std::string layer_name) {
  if (IsLayersFull()) {
    return false;
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

int CollisionFilterRegistrar::LookUpLayerId(std::string layer_name) {
  if (layer_id_table_.count(layer_name) == 0) {
    return LAYER_NOT_EXIST;
  }
  return layer_id_table_[layer_name];
}

}; // namespace flatland_server
