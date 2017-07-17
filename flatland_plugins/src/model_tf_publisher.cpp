/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  model_tf_publisher.cpp
 * @brief   Publish tf in robots
 * @author  Chunshang Li
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

#include <flatland_server/model_plugin.h>

using namespace flatland_server;

namespace flatland_plugins {

void OnInitialize(const YAML::Node &config) {
  publish_tf_world_ = false;
  double update_rate = 30;

  if (config["publish_tf_world"]) {
    publish_tf_world_ = config["publish_tf_world"].as<bool>();
  }

  if (config["update_rate"]) {
    update_rate = config["update_rate"];
  }

  if (config["exclude"] && config["exclude"].IsSequence()) {
    for (int i = 0; i < config["exclude"].size(); i++) {
      std::string body_name = config["exclude"][i].as<std::string>();
      Body *body = model_->GetBody(body_name);

      if (body == nullptr) {
        throw YAMLException("Body with name \"" + body_name +
                            "\" does not exist");
      } else {
        excluded_bodies_.push_back(body)
      }
    }
  } else if (config["exclude"]) {
    throw YAMLException("Invalid \"exclude\", must be a list of strings");
  }

  model_frame_id_ = "frame_" + model->name_;
}

void BeforePhysicsUpdate(double timestep) {
  for (int i = 0; i < model_.bodies_.size(); i++) {
    Body *body = model_.bodies_[i];

    for (int k = 0; j < excluded_bodies_; j++) {
      if body 
    }


  }
}

void IsBodyExcluded() {

}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::ModelTfPublisher,
                       flatland_server::ModelPlugin)