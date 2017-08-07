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
#include <flatland_server/yaml_reader.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <boost/algorithm/string/join.hpp>

using namespace flatland_server;

namespace flatland_plugins {

void ModelTfPublisher::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  // default values
  publish_tf_world_ = reader.Get<bool>("publish_tf_world", false);
  world_frame_id_ = reader.Get<std::string>("world_frame_id", "map");
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());
  tf_prefix_ = model_->GetNameSpace();
  std::string ref_body_name = reader.Get<std::string>("reference", "");
  std::vector<std::string> excluded_body_names =
      reader.GetList<std::string>("exclude", {}, -1, -1);
  reader.EnsureAccessedAllKeys();

  if (ref_body_name.size() != 0) {
    reference_body_ = model_->GetBody(ref_body_name);

    if (reference_body_ == nullptr) {
      throw YAMLException("Body with name \"" + ref_body_name +
                          "\" does not exist");
    }
  } else {
    // defaults to the first body, the reference body has no effect on the
    // final result, but it changes how the TF would look
    reference_body_ = model_->bodies_[0];
  }

  for (int i = 0; i < excluded_body_names.size(); i++) {
    Body *body = model_->GetBody(excluded_body_names[i]);

    if (body == nullptr) {
      throw YAMLException("Body with name \"" + excluded_body_names[i] +
                          "\" does not exist");
    } else {
      excluded_bodies_.push_back(body);
    }
  }

  update_timer_.SetRate(update_rate_);

  ROS_INFO_NAMED(
      "ModelTfPublisher",
      "Initialized with params: reference(%s %p) "
      "publish_tf_world(%d) world_frame_id(%s) update_rate(%f), exclude({%s})",
      reference_body_->name_.c_str(), reference_body_, publish_tf_world_,
      world_frame_id_.c_str(), update_rate_,
      boost::algorithm::join(excluded_body_names, ",").c_str());
}

void ModelTfPublisher::BeforePhysicsStep(const Timekeeper &timekeeper) {
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  Eigen::Matrix3f ref_tf_m;  ///< for storing TF from world to the ref. body
  Eigen::Matrix3f rel_tf;    ///< for storing TF from ref. body to other bodies

  // fill the world to ref. body TF with data from Box2D
  const b2Transform &r = reference_body_->physics_body_->GetTransform();
  ref_tf_m << r.q.c, -r.q.s, r.p.x, r.q.s, r.q.c, r.p.y, 0, 0, 1;

  geometry_msgs::TransformStamped tf_stamped;
  tf_stamped.header.stamp = ros::Time::now();

  // loop through the bodies to calculate TF, and ignores excluded bodies
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

    // Get transformation of body w.r.t to the world
    const b2Transform &b = body->physics_body_->GetTransform();
    Eigen::Matrix3f body_tf_m;
    body_tf_m << b.q.c, -b.q.s, b.p.x, b.q.s, b.q.c, b.p.y, 0, 0, 1;

    // this calculates the transformation from the reference body to the
    // other body. It is needed because Box2D only provides position and
    // angle of bodies w.r.t to the world
    rel_tf = ref_tf_m.inverse() * body_tf_m;

    // obtain the yaw from the transformation matrice, we can use any one
    // of the 4 trigonometric elements in matrix
    double cosine = rel_tf(0, 0);
    double yaw;

    // There is a chance the cosine is slightly larger than 1.0f due to
    // floating point error, -1 <= cos(x) <= 1, assign acos(1) = 0 to yaw.
    // We limit this floating point error to 1e-3, if the error is larger
    // something else is probably wrong
    if (cosine >= -1 && cosine <= 1) {
      yaw = acos(cosine);
    } else if (fabs(cosine) - 1 < 1e-3) {
      yaw = 0.0;
    } else {
      ROS_ERROR_NAMED("ModelTfPublisher", "fabs(cos(x)) - 1 > 1e-3");
    }

    // publish TF
    tf_stamped.header.frame_id =
        tf::resolve(tf_prefix_, reference_body_->name_);
    tf_stamped.child_frame_id = tf::resolve(tf_prefix_, body->name_);
    tf_stamped.transform.translation.x = rel_tf(0, 2);
    tf_stamped.transform.translation.y = rel_tf(1, 2);
    tf_stamped.transform.translation.z = 0;
    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(tf_stamped);
  }

  // publish world TF if necessary
  if (publish_tf_world_) {
    const b2Vec2 &p = reference_body_->physics_body_->GetPosition();
    double yaw = reference_body_->physics_body_->GetAngle();

    tf_stamped.header.frame_id = world_frame_id_;
    tf_stamped.child_frame_id = tf::resolve(tf_prefix_, reference_body_->name_);
    tf_stamped.transform.translation.x = p.x;
    tf_stamped.transform.translation.y = p.y;
    tf_stamped.transform.translation.z = 0;
    tf::Quaternion q;
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