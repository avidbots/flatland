/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	Bicycle.cpp
 * @brief   Bicycle plugin
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

#include <Box2D/Box2D.h>
#include <flatland_plugins/bicycle.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

//
// Usage:
//
//      Include the following in the model's Yaml file
//
//      plugins:
//          - type: Bicycle # plugin class name
//          name: bicycle # for registering list of plugins
//          body: base
//          origin: [0, 0, 0] # w.r.t body origin, where the twist drive is
//          applied
//
//      The density of the chassis main fixture must be set to 100000.0
//
namespace flatland_plugins {

// This code simply moves the origin of the chassis
// and prints it out in yaml format
/*
void Bicycle::addMeA(double offset) {
    b2Vec2 vert[6];
    vert[0].Set(-0.337,-1.03);
    vert[1].Set(-0.337, .07983);
    vert[2].Set(-.16111, .30);
    vert[3].Set(.16111, .30);
    vert[4].Set(0.337, .07983);
    vert[5].Set(0.337,-1.03);
    for(int i=0; i<8 ;i++) {
        std::cout << "          ["<< vert[i].x << ", " << vert[i].y+offset <<
"]," << std::endl;
    }
    std::cout << " " << std::endl;
}

void Bicycle::addMeB(double offset) {
    b2Vec2 vertA[8];
    vertA[0].Set(-0, 0);
    vertA[1].Set(-0.337, 0);
    vertA[2].Set(-0.337, 1.13983);
    vertA[3].Set(-0.31971, 1.18634);
    vertA[4].Set(-0.27569, 1.24002);
    vertA[5].Set(-0.18131, 1.3056);
    vertA[6].Set(-0.11, 1.33058);
    vertA[7].Set(-0,  1.33058);

    b2Vec2 vertB[8];
    vertB[0].Set(-0, 0);
    vertB[1].Set(0,  1.33058);
    vertB[2].Set(0.11, 1.33058);
    vertB[3].Set(0.18131, 1.3056);
    vertB[4].Set(0.27569, 1.24002);
    vertB[5].Set(0.31971, 1.18634);
    vertB[6].Set(0.337, 1.13983);
    vertB[7].Set(0.337, 0);

    for(int i=0; i<8 ;i++) {
        std::cout << "          ["<< vertA[i].x << ", " << vertA[i].y+offset <<
"]," << std::endl;
    }
    std::cout << " " << std::endl;
    for(int i=0; i<8 ;i++) {
        std::cout << "          ["<< vertB[i].x << ", " << vertB[i].y+offset <<
"]," << std::endl;
    }
    std::cout << " " << std::endl;

}
*/

void Bicycle::OnInitialize(const YAML::Node& config) {
  ROS_INFO_NAMED("BicyclePlugin", "Bicycle Initialized");
  robotAngle = 0.0;
  robotAlpha = 0.0;
  robotPosition = b2Vec2(0, 0);
  count = 0;

  // get the robot pointer
  robot = model_->GetBody("base")->physics_body_;

  // subscribe to the cmd_vel topic
  sub = nh_.subscribe("/cmd_vel", 0, &Bicycle::twistCallback, this);

  Bicycle::createFrontWheel();

  // Bicycle::addMe2(-1.03);

  robotIsInMotion = false;
  modelIsDynamic = true;
}

//
// rotates a b2Vec2 point about the origin by angle radians
//
b2Vec2 rotateVertex(b2Vec2 vertex, double angle) {
  return b2Vec2(vertex.x * cos(angle) + vertex.y * sin(angle),
                vertex.x * -sin(angle) + vertex.y * cos(angle));
}

void Bicycle::createFrontWheel() {
  // create the shape
  b2PolygonShape polygon;
  const double height = .175;
  const double width = 0.05;
  const double yOffset = 0.0;

  // setup the vertices
  b2Vec2 vertices[4];
  vertices[0] =
      rotateVertex(b2Vec2(-width / 2, -height / 2 + yOffset), robotAlpha);
  vertices[1] =
      rotateVertex(b2Vec2(width / 2, -height / 2 + yOffset), robotAlpha);
  vertices[2] =
      rotateVertex(b2Vec2(width / 2, height / 2 + yOffset), robotAlpha);
  vertices[3] =
      rotateVertex(b2Vec2(-width / 2, height / 2 + yOffset), robotAlpha);

  polygon.Set(vertices, 4);

  // create the fixture definition
  b2FixtureDef fixtureDef;

  // define no collision between robot and wheel
  fixtureDef.filter.categoryBits = 0x0001;
  fixtureDef.filter.maskBits = ~0x0001;

  // bind the shape to the fixture definition
  fixtureDef.shape = &polygon;

  // create the fixture (save pointer)
  b2Body* base_body = model_->GetBody("base")->physics_body_;
  frontWheelFixture = base_body->CreateFixture(&fixtureDef);
}

void Bicycle::destroyFrontWheel() { robot->DestroyFixture(frontWheelFixture); }

void Bicycle::recreateFrontWheel() {
  destroyFrontWheel();
  createFrontWheel();
}

double Bicycle::calculateDelta(double distance) {
  double r;      // turn diameter
  double R;      // distance along wheel axis to ICC
  double B;      // distance from turning wheel to midpoint between rear wheels
  double delta;  // angle to rotate wheel about ICC

  if (fabs(robotAlpha) < 0.01 * b2_pi / 180.0) {
    delta = 0.0;
  } else {
    // calculate the wheel rotation angle about ICC
    B = 0.83;
    R = B / tan(robotAlpha);
    r = R / cos(robotAlpha);
    delta = atan(distance / r);
  }

  return delta;
}

void Bicycle::BeforePhysicsStep(double timestep) {
  time_step = timestep * speedFactor;

  robot = model_->GetBody("base")->physics_body_;

  // This bit of code drives the robot in (constant) circles forward and back
  /*
  count++;
  if (count < 35) {
    omega = 1.0;
    velocity = 0.0;
  } else {
    if (count > 356) {
      omega = 0.0;
      velocity = -0.5;

    } else {
      omega = 0.0;
      velocity = 0.5;
    }
  }
  if (count > 700) {
    count = 30;
  }
  ROS_INFO_STREAM("  velocity:" << velocity << "  omega:" << omega
                                << "  angle:" << robot->GetAngle() * 180 / b2_pi
                                << "  count:" << count);
  */

  // robotIsInMotion = true;
  applyVelocity();

  flatland_server::DebugVisualization::Get().Reset("diffbody");
  flatland_server::DebugVisualization::Get().Visualize("diffbody", robot, 1.0,
                                                       1.0, 1.0, 0.5);

  recreateFrontWheel();
}

void Bicycle::twistCallback(const geometry_msgs::Twist& msg) {
  velocity = msg.linear.x;
  omega = msg.angular.z;
  robotIsInMotion = true;
}

void Bicycle::applyVelocity() {
  static b2Vec2 lastStep, lastPos, thisPos;
  double distance, delta, travelLimit = 85.0 * b2_pi / 180.0;

  if (robotAlpha > travelLimit) {
    if (omega < 0.0) {
      robotAngle -= omega * time_step;
      robotAlpha += omega * time_step;
    }
  }

  if (robotAlpha < -travelLimit) {
    if (omega > 0.0) {
      robotAngle -= omega * time_step;
      robotAlpha += omega * time_step;
    }
  }

  // integrate the angual velocity
  robotAngle += omega * time_step;
  robotAlpha -= omega * time_step;

  if (modelIsDynamic) {
    //
    // Dynamic model
    //
    // if (!robotIsInMotion) {
    //
    // future: decelerate/stop robot
    //
    //
    // robot->SetLinearVelocity(b2Vec2(0.0,0.0));
    // robot->SetAngularVelocity(0.0);
    //} else {
    double angle2 = robot->GetAngle();
    thisPos = robot->GetPosition();
    lastStep = thisPos - lastPos;

    distance = sqrt(lastStep.x * lastStep.x + lastStep.y * lastStep.y);
    b2Vec2 linearVelocity;

    double ff = 250.0;
    linearVelocity.x = -velocity * sin(angle2 - robotAlpha) * ff * time_step;
    linearVelocity.y = velocity * cos(angle2 - robotAlpha) * ff * time_step;

    if (velocity != 0.0) {
      robot->SetLinearVelocity(linearVelocity);

      delta = calculateDelta(distance);
      if (velocity > 0) {
        robot->SetTransform(thisPos, angle2 - delta);

      } else {
        robot->SetTransform(thisPos, angle2 + delta);
      }
      robotAngle -= delta;

      // set the next robot position
      robotPosition = thisPos;

    } else {
      robot->SetLinearVelocity(b2Vec2(0, 0));
    }
    lastPos = thisPos;
    //}
    robotIsInMotion = false;
  } else {
    //
    // Kinematic model
    //
    // integrate the linear velocity
    distance = velocity * time_step;

    // set the next robot position
    robotPosition.x -= velocity * sin(robotAngle) * time_step;
    robotPosition.y += velocity * cos(robotAngle) * time_step;

    delta = calculateDelta(distance);
    robotAngle -= delta;
    robot->SetTransform(robotPosition, (robotAngle + robotAlpha));
  }

  // ROS_INFO_STREAM("  x:" << robotPosition.x << "  y:" << robotPosition.y);
  // ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<"
  // angular="<<msg.angular.z);
  // ROS_INFO_STREAM(" robotAngle="<<robotAngle*180.0/b2_pi<<"  delta="<<
  // delta*180.0/b2_pi<<"  robotAlpha:"<<robotAlpha*180.0/b2_pi);
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Bicycle, flatland_server::ModelPlugin)