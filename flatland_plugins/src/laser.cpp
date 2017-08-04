/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  laser.cpp
 * @brief   Laser plugin
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

#include <flatland_plugins/laser.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <boost/algorithm/string/join.hpp>
#include <cmath>
#include <limits>

using namespace flatland_server;

namespace flatland_plugins {

void Laser::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);

  update_timer_.SetRate(update_rate_);
  scan_publisher = nh_.advertise<sensor_msgs::LaserScan>(topic_, 1);

  // construct the body to laser transformation matrix once since it never
  // changes
  double c = cos(origin_.theta);
  double s = sin(origin_.theta);
  double x = origin_.x, y = origin_.y;
  m_body_to_laser_ << c, -s, x, s, c, y, 0, 0, 1;

  num_laser_points_ = std::lround((max_angle_ - min_angle_) / increment_) + 1;

  // initialize size for the matrix storing the laser points
  m_laser_points_ = Eigen::MatrixXf(3, num_laser_points_);
  m_world_laser_points_ = Eigen::MatrixXf(3, num_laser_points_);
  v_zero_point_ << 0, 0, 1;

  // pre-calculate the laser points w.r.t to the laser frame, since this never
  // changes
  for (int i = 0; i < num_laser_points_; i++) {
    float angle = min_angle_ + i * increment_;

    float x = range_ * cos(angle);
    float y = range_ * sin(angle);

    m_laser_points_(0, i) = x;
    m_laser_points_(1, i) = y;
    m_laser_points_(2, i) = 1;
  }

  // initialize constants in the laser scan message
  laser_scan_.angle_min = min_angle_;
  laser_scan_.angle_max = max_angle_;
  laser_scan_.angle_increment = increment_;
  laser_scan_.time_increment = 0;
  laser_scan_.scan_time = 0;
  laser_scan_.range_min = 0;
  laser_scan_.range_max = range_;
  laser_scan_.ranges.resize(num_laser_points_);
  laser_scan_.intensities.resize(0);
  laser_scan_.header.seq = 0;
  laser_scan_.header.frame_id = tf::resolve(model_->namespace_, frame_id_);

  // Broadcast transform between the body and laser
  tf::Quaternion q;
  q.setRPY(0, 0, origin_.theta);

  static_tf.header.frame_id = tf::resolve(model_->namespace_, body_->name_);
  static_tf.child_frame_id = tf::resolve(model_->namespace_, frame_id_);
  static_tf.transform.translation.x = origin_.x;
  static_tf.transform.translation.y = origin_.y;
  static_tf.transform.translation.z = 0;
  static_tf.transform.rotation.x = q.x();
  static_tf.transform.rotation.y = q.y();
  static_tf.transform.rotation.z = q.z();
  static_tf.transform.rotation.w = q.w();

  ROS_INFO_NAMED("LaserPlugin", "Laser %s initialized", name_.c_str());
}

void Laser::BeforePhysicsStep(const Timekeeper &timekeeper) {
  // keep the update rate
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  // get the transformation matrix from the world to the body, and get the
  // world to laser frame transformation matrix by multiplying the world to body
  // and body to laser
  const b2Transform &t = body_->physics_body_->GetTransform();
  m_world_to_body_ << t.q.c, -t.q.s, t.p.x, t.q.s, t.q.c, t.p.y, 0, 0, 1;
  m_world_to_laser_ = m_world_to_body_ * m_body_to_laser_;

  // Get the laser points in the world frame by multiplying the laser points in
  // the laser frame to the transformation matrix from world to laser frame
  m_world_laser_points_ = m_world_to_laser_ * m_laser_points_;
  // Get the (0, 0) point in the laser frame
  v_world_laser_origin_ = m_world_to_laser_ * v_zero_point_;

  // Conver to Box2D data types
  b2Vec2 laser_point;
  b2Vec2 laser_origin_point(v_world_laser_origin_(0), v_world_laser_origin_(1));

  // loop through the laser points and call the Box2D world raycast
  for (int i = 0; i < num_laser_points_; ++i) {
    laser_point.x = m_world_laser_points_(0, i);
    laser_point.y = m_world_laser_points_(1, i);

    did_hit_ = false;

    model_->physics_world_->RayCast(this, laser_origin_point, laser_point);

    if (!did_hit_) {
      laser_scan_.ranges[i] = NAN;
    } else {
      laser_scan_.ranges[i] = fraction_ * range_;
    }
  }

  laser_scan_.header.stamp = ros::Time::now();
  scan_publisher.publish(laser_scan_);

  static_tf.header.stamp = ros::Time::now();
  tf_broadcaster.sendTransform(static_tf);
}

float Laser::ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                           const b2Vec2 &normal, float fraction) {
  // only register hit in the specified layers
  if (!(fixture->GetFilterData().categoryBits & layers_bits_)) {
    return -1.0f;  // return -1 to ignore this hit
  }

  did_hit_ = true;
  fraction_ = fraction;

  return fraction;
}

void Laser::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  topic_ = reader.Get<std::string>("topic", "scan");
  frame_id_ = reader.Get<std::string>("frame", name_);
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));
  range_ = reader.Get<double>("range");
  std::vector<std::string> layers =
      reader.GetList<std::string>("layers", {"all"}, -1, -1);

  YamlReader angle_reader = reader.Subnode("angle", YamlReader::MAP);
  min_angle_ = angle_reader.Get<double>("min");
  max_angle_ = angle_reader.Get<double>("max");
  increment_ = angle_reader.Get<double>("increment");

  angle_reader.EnsureAccessedAllKeys();
  reader.EnsureAccessedAllKeys();

  if (max_angle_ < min_angle_) {
    throw YAMLException("Invalid \"angle\" params, must have max > min");
  }

  body_ = model_->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  std::vector<std::string> invalid_layers;
  layers_bits_ = model_->cfr_->GetCategoryBits(layers, &invalid_layers);
  if (!invalid_layers.empty()) {
    throw YAMLException("Cannot find layer(s): {" +
                        boost::algorithm::join(invalid_layers, ",") + "}");
  }

  ROS_INFO_NAMED("LaserPlugin",
                 "Laser %s params: topic(%s) body(%s %p) origin(%f,%f,%f) "
                 "frame_id(%s) update_rate(%f) range(%f) angle_min(%f) "
                 "angle_max(%f) angle_increment(%f) layers(0x%u {%s})",
                 name_.c_str(), topic_.c_str(), body_name.c_str(), body_,
                 origin_.x, origin_.y, origin_.theta, frame_id_.c_str(),
                 update_rate_, range_, min_angle_, max_angle_, increment_,
                 layers_bits_, boost::algorithm::join(layers, ",").c_str());
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Laser, flatland_server::ModelPlugin)