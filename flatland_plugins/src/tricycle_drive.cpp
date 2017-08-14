/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	tricycle_drive.cpp
 * @brief   tricycle plugin
 * @author  Mike Brousseau
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

#include <Box2D/Box2D.h>
#include <flatland_plugins/tricycle_drive.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace flatland_plugins {

void TricycleDrive::OnInitialize(const YAML::Node& config) {
  YamlReader r(config);

  // load all the parameters
  string body_name = r.Get<string>("body");
  string front_wj_name = r.Get<string>("front_wheel_joint");
  string rear_left_wj_name = r.Get<string>("rear_left_wheel_joint");
  string rear_right_wj_name = r.Get<string>("rear_right_wheel_joint");
  string odom_frame_id = r.Get<string>("odom_frame_id", "odom");

  string twist_topic = r.Get<string>("twist_sub", "cmd_vel");
  string odom_topic = r.Get<string>("odom_pub", "odometry/filtered");
  string ground_truth_topic =
      r.Get<string>("ground_truth_pub", "odometry/ground_truth");

  // noise are in the form of linear x, linear y, angular variances
  vector<double> odom_twist_noise =
      r.GetList<double>("odom_twist_noise", {0, 0, 0}, 3, 3);
  vector<double> odom_pose_noise =
      r.GetList<double>("odom_pose_noise", {0, 0, 0}, 3, 3);

  double pub_rate =
      r.Get<double>("pub_rate", numeric_limits<double>::infinity());
  update_timer_.SetRate(pub_rate);

  // by default the covariance diagonal is the variance of actual noise
  // generated, non-diagonal elements are zero assuming the noises are
  // independent, we also don't care about linear z, angular x, and angular y
  array<double, 36> odom_pose_covar_default = {0};
  odom_pose_covar_default[0] = odom_pose_noise[0];
  odom_pose_covar_default[7] = odom_pose_noise[1];
  odom_pose_covar_default[35] = odom_pose_noise[2];

  array<double, 36> odom_twist_covar_default = {0};
  odom_twist_covar_default[0] = odom_twist_noise[0];
  odom_twist_covar_default[7] = odom_twist_noise[1];
  odom_twist_covar_default[35] = odom_twist_noise[2];

  auto odom_twist_covar =
      r.GetArray<double, 36>("odom_twist_covariance", odom_twist_covar_default);
  auto odom_pose_covar =
      r.GetArray<double, 36>("odom_pose_covariance", odom_pose_covar_default);

  r.EnsureAccessedAllKeys();

  // Get the bodies and joints from names, throw if not found
  body_ = model_->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }

  front_wj_ = model_->GetJoint(front_wj_name);
  if (front_wj_ == nullptr) {
    throw YAMLException("Joint with name " + Q(front_wj_name) +
                        " does not exist");
  }

  rear_left_wj_ = model_->GetJoint(rear_left_wj_name);
  if (rear_left_wj_ == nullptr) {
    throw YAMLException("Joint with name " + Q(rear_left_wj_name) +
                        " does not exist");
  }

  rear_right_wj_ = model_->GetJoint(rear_right_wj_name);
  if (rear_right_wj_ == nullptr) {
    throw YAMLException("Joint with name " + Q(rear_right_wj_name) +
                        " does not exist");
  }

  // validate the that joints fits the assumption of the robot model and
  // calculate rear wheel separation and wheel base
  ComputeJoints();

  // publish and subscribe to topics
  twist_sub_ =
      nh_.subscribe(twist_topic, 1, &TricycleDrive::TwistCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 1);
  ground_truth_pub_ = nh_.advertise<nav_msgs::Odometry>(ground_truth_topic, 1);

  // init the values for the messages
  ground_truth_msg_.header.frame_id = odom_frame_id;
  ground_truth_msg_.child_frame_id = body_->name_;
  ground_truth_msg_.twist.covariance.fill(0);
  ground_truth_msg_.pose.covariance.fill(0);
  odom_msg_ = ground_truth_msg_;

  // copy from array to boost array
  for (int i = 0; i < 36; i++) {
    odom_msg_.twist.covariance[i] = odom_twist_covar[i];
    odom_msg_.pose.covariance[i] = odom_pose_covar[i];
  }

  // init the random number generators
  random_device rd;
  rng_ = default_random_engine(rd());
  for (int i = 0; i < 3; i++) {
    // variance is standard deviation squared
    noise_gen_[i] = normal_distribution<double>(0.0, sqrt(odom_pose_noise[i]));
  }

  for (int i = 0; i < 3; i++) {
    noise_gen_[i + 3] =
        normal_distribution<double>(0.0, sqrt(odom_twist_noise[i]));
  }

  ROS_DEBUG_NAMED(
      "TricycleDrive",
      "Initialized with params body(%p %s) front_wj(%p %s) "
      "rear_left_wj(%p %s) rear_right_wj(%p %s) "
      "odom_frame_id(%s) twist_sub(%s) odom_pub(%s) "
      "ground_truth_pub(%s) odom_pose_noise({%f,%f,%f}) "
      "odom_twist_noise({%f,%f,%f}) pub_rate(%f)\n",
      body_, body_->GetName().c_str(), front_wj_, front_wj_->GetName().c_str(),
      rear_left_wj_, rear_left_wj_->GetName().c_str(), rear_right_wj_,
      rear_right_wj_->GetName().c_str(), odom_frame_id.c_str(),
      twist_topic.c_str(), odom_topic.c_str(), ground_truth_topic.c_str(),
      odom_pose_noise[0], odom_pose_noise[1], odom_pose_noise[2],
      odom_twist_noise[0], odom_twist_noise[1], odom_twist_noise[2], pub_rate);
}

void TricycleDrive::ComputeJoints() {
  auto get_anchor = [&](Joint* joint) {

    b2Vec2 wheel_anchor;  ///< wheel anchor point, must be (0,0)
    b2Vec2 body_anchor;   ///< body anchor point
    Body* wheel_body;

    // ensure one of the body is the main body for the odometry
    if (joint->physics_joint_->GetBodyA()->GetUserData() == body_) {
      wheel_anchor = joint->physics_joint_->GetAnchorB();
      body_anchor = joint->physics_joint_->GetAnchorA();
    } else if (joint->physics_joint_->GetBodyB()->GetUserData() == body_) {
      wheel_anchor = joint->physics_joint_->GetAnchorA();
      body_anchor = joint->physics_joint_->GetAnchorB();
    } else {
      throw YAMLException("Joint " + Q(joint->GetName()) +
                          " does not anchor on body " + Q(body_->GetName()));
    }

    // convert anchor is global coordinates to local body coordinates
    wheel_anchor = body_->physics_body_->GetLocalPoint(wheel_anchor);
    body_anchor = body_->physics_body_->GetLocalPoint(body_anchor);

    // ensure the joint is anchored at (0,0) of the wheel_body
    if (fabs(wheel_anchor.x) > 1e-5 || fabs(wheel_anchor.y) > 1e-5) {
      throw YAMLException("Joint " + Q(joint->GetName()) +
                          " must have its wheel anchored point at (0, 0)");
    }

    return body_anchor;
  };

  // joints must be of expected type
  if (front_wj_->physics_joint_->GetType() != e_revoluteJoint) {
    throw YAMLException("Front wheel joint must be a revolute joint");
  }

  if (rear_left_wj_->physics_joint_->GetType() != e_weldJoint) {
    throw YAMLException("Rear left wheel joint must be a weld joint");
  }

  if (rear_right_wj_->physics_joint_->GetType() != e_weldJoint) {
    throw YAMLException("Rear right wheel joint must be a weld joint");
  }

  // Get the anchor points of wheels on the body, the front wheel must be
  // at (0,0) of the body
  b2Vec2 front_anchor = get_anchor(front_wj_);
  b2Vec2 rear_left_anchor = get_anchor(rear_left_wj_);
  b2Vec2 rear_right_anchor = get_anchor(rear_right_wj_);

  if (fabs(front_anchor.x) > 1e-5 || fabs(front_anchor.y) > 1e-5) {
    throw YAMLException(
        "Front wheel joint must have its body anchored at (0, 0)");
  }

  // calculate the wheelbase and axeltrack. We also need to verify that
  // the rear_center is at the perpendicular intersection between the rear axel
  // and the front wheel anchor
  rear_center_ = 0.5 * (rear_left_anchor + rear_right_anchor);

  // find the perpendicular intersection between line segment given by (x1, y1)
  // and (x2, y2) and a point (x3, y3).
  double x1 = rear_left_anchor.x, y1 = rear_left_anchor.y,
         x2 = rear_right_anchor.x, y2 = rear_right_anchor.y,
         x3 = front_anchor.x, y3 = front_anchor.y;

  double k = ((y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1)) /
             ((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
  double x4 = x3 - k * (y2 - y1);
  double y4 = y3 + k * (x2 - x1);

  // check (x4, y4) equals to rear_center_
  if (fabs(x4 - rear_center_.x) > 1e-5 || fabs(y4 - rear_center_.y) > 1e-5) {
    throw YAMLException(
        "The mid point between the rear wheel anchors on the body must equal "
        "the perpendicular intersection between the rear axel (line segment "
        "between rear anchors) and the front wheel anchor");
  }

  // track is the separation between the rear two wheels, which is simply the
  // distance between the rear two wheels
  axel_track_ = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

  // wheel base is the perpendicular distance between the rear axel and the
  // front wheel
  wheelbase_ = sqrt((x4 - x3) * (x4 - x3) + (y4 - y3) * (y4 - y3));
}

void TricycleDrive::BeforePhysicsStep(const Timekeeper& timekeeper) {}

void TricycleDrive::TwistCallback(const geometry_msgs::Twist& msg) {
  twist_msg_ = msg;
}

// void TricycleDrive::ApplyVelocity() {
//   static b2Vec2 last_step, last_pos, this_pos;
//   double distance, delta, travelLimit = 85.0 * b2_pi / 180.0;

//   if (robot_alpha > travelLimit) {
//     if (omega < 0.0) {
//       robot_angle -= omega * time_step;
//       robot_alpha += omega * time_step;
//     }
//   }

//   if (robot_alpha < -travelLimit) {
//     if (omega > 0.0) {
//       robot_angle -= omega * time_step;
//       robot_alpha += omega * time_step;
//     }
//   }

//   // integrate the angual velocity
//   robot_angle += omega * time_step;
//   robot_alpha -= omega * time_step;

//   if (model_is_dynamic) {
//     //
//     // Dynamic model
//     //
//     // future: decelerate/stop robot
//     //
//     //
//     // robot->SetLinearVelocity(b2Vec2(0.0,0.0));
//     // robot->SetAngularVelocity(0.0);
//     double angle2 = robot->GetAngle();
//     this_pos = robot->GetPosition();
//     last_step = this_pos - last_pos;

//     distance = sqrt(last_step.x * last_step.x + last_step.y * last_step.y);
//     b2Vec2 linearVelocity;

//     double ff = 250.0;  // fudge factor to make dynamic like kinematic
//     linearVelocity.x = -velocity * sin(angle2 - robot_alpha) * ff *
//     time_step;
//     linearVelocity.y = velocity * cos(angle2 - robot_alpha) * ff * time_step;

//     if (velocity != 0.0) {
//       robot->SetLinearVelocity(linearVelocity);

//       delta = CalculateDelta(distance);
//       if (velocity > 0) {
//         robot->SetTransform(this_pos, angle2 - delta);

//       } else {
//         robot->SetTransform(this_pos, angle2 + delta);
//       }
//       robot_angle -= delta;

//       // set the next robot position
//       robot_position = this_pos;

//     } else {
//       robot->SetLinearVelocity(b2Vec2(0, 0));
//     }
//     last_pos = this_pos;
//   } else {
//     //
//     // Kinematic model
//     //
//     // integrate the linear velocity
//     distance = velocity * time_step;

//     // set the next robot position
//     robot_position.x -= velocity * sin(robot_angle) * time_step;
//     robot_position.y += velocity * cos(robot_angle) * time_step;

//     delta = CalculateDelta(distance);
//     robot_angle -= delta;
//     robot->SetTransform(robot_position, (robot_angle + robot_alpha));
//   }
// }
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::TricycleDrive,
                       flatland_server::ModelPlugin)