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

#include <flatland_plugins/model_tf_publisher.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <boost/algorithm/string/join.hpp>

using namespace flatland_server;

namespace flatland_plugins {

void ModelTfPublisher::OnInitialize(const YAML::Node &config) {
  publish_tf_world_ = false;
  double update_rate = 30;
  world_frame_id_ = "map";

  if (config["reference"]) {
    std::string body_name = config["reference"].as<std::string>();
    reference_body_ = model_->GetBody(body_name);

    if (reference_body_ == nullptr) {
      throw YAMLException("Body with name \"" + body_name +
                          "\" does not exist");
    }

  } else {
    reference_body_ = model_->bodies_[0];
  }

  if (config["world_frame_id"]) {
    world_frame_id_ = config["world_frame_id"].as<std::string>();
  }

  if (config["publish_tf_world"]) {
    publish_tf_world_ = config["publish_tf_world"].as<bool>();
  }

  if (config["update_rate"]) {
    update_rate = config["update_rate"].as<double>();
  }

  std::vector<std::string> excluded_body_names;
  if (config["exclude"] && config["exclude"].IsSequence()) {
    for (int i = 0; i < config["exclude"].size(); i++) {
      std::string body_name = config["exclude"][i].as<std::string>();
      excluded_body_names.push_back(body_name);
      Body *body = model_->GetBody(body_name);

      if (body == nullptr) {
        throw YAMLException("Body with name \"" + body_name +
                            "\" does not exist");
      } else {
        excluded_bodies_.push_back(body);
      }
    }
  } else if (config["exclude"]) {
    throw YAMLException("Invalid \"exclude\", must be a list of strings");
  }

  ROS_INFO_NAMED("ModelTfPublisher",
                 "Initialized with params: reference(%s %p) "
                 "publish_tf_world(%d) update_rate(%f), exclude(%s)",
                 reference_body_->name_.c_str(), reference_body_,
                 publish_tf_world_, update_rate,
                 boost::algorithm::join(excluded_body_names, ",").c_str());
}

void ModelTfPublisher::BeforePhysicsStep(double timestep) {
  const b2Transform &r = reference_body_->physics_body_->GetTransform();
  Eigen::Matrix3f ref_tf_m;
  Eigen::Matrix3f rel_tf;
  ref_tf_m << r.q.c, -r.q.s, r.p.x, /**/ r.q.s, r.q.c, r.p.y, /**/ 0, 0, 1;
  // std::cout << ref_tf_m << std::endl;
  geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = ros::Time::now();

  for (int i = 0; i < model_->bodies_.size(); i++) {
    Body *body = model_->bodies_[i];
    bool is_excluded = false;

    for (int j = 0; j < excluded_bodies_.size(); j++) {
      if (body == excluded_bodies_[j]) {
        is_excluded = true;
      }
    }

    if (is_excluded || body == reference_body_) {
      continue;
    }

    // start publishing the transformations
    const b2Transform &b = body->physics_body_->GetTransform();
    Eigen::Matrix3f body_tf_m;
    body_tf_m << b.q.c, -b.q.s, b.p.x, b.q.s, b.q.c, b.p.y, 0, 0, 1;
    // std::cout << body_tf_m << std::endl;

    rel_tf = ref_tf_m.inverse() * body_tf_m;

    // std::cout << rel_tf << std::endl;

    // printf("%.15f???\n", rel_tf(0, 0));


    double cosine = rel_tf(0, 0);
    double yaw;
    if (cosine > 1.0) {
      yaw = 0.0;
    } else {
      yaw = acos(cosine);
    }

    tf_stamped.header.frame_id = reference_body_->name_;
    tf_stamped.child_frame_id = body->name_;
    tf_stamped.transform.translation.x = rel_tf(0, 2);
    tf_stamped.transform.translation.y = rel_tf(1, 2);
    tf_stamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    // printf("%f, %f, %f\n", rel_tf(0, 2), rel_tf(1, 2), yaw);
    // printf("yaw?\n");

    tf_broadcaster.sendTransform(tf_stamped);
  }

  if (publish_tf_world_) {
    const b2Vec2 &p = reference_body_->physics_body_->GetPosition();
    double yaw = reference_body_->physics_body_->GetAngle();

    // tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = world_frame_id_;
    tf_stamped.child_frame_id = reference_body_->name_;
    tf_stamped.transform.translation.x = p.x;
    tf_stamped.transform.translation.y = p.y;
    tf_stamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(tf_stamped);
  }
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::ModelTfPublisher,
                       flatland_server::ModelPlugin)