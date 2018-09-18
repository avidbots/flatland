/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	debug_visualization.h
 * @brief Transform box2d types into published visualization messages
 * @author Joseph Duchesne
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

#ifndef FLATLAND_SERVER_DEBUG_VISUALIZATION_H
#define FLATLAND_SERVER_DEBUG_VISUALIZATION_H

#include <Box2D/Box2D.h>
#include <flatland_msgs/DebugTopicList.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <map>
#include <string>
#include <vector>

#include "flatland_server/body.h"
#include "flatland_server/timekeeper.h"

namespace flatland_server {
struct DebugTopic {
  ros::Publisher publisher;
  bool needs_publishing;
  visualization_msgs::MarkerArray markers;
};

class DebugVisualization {
 private:
  DebugVisualization();

 public:
  std::map<std::string, DebugTopic> topics_;
  ros::NodeHandle node_;
  ros::Publisher topic_list_publisher_;

  /**
   * @brief Return the singleton object
   */
  static DebugVisualization& Get();

  /**
   * @brief Publish all marker array topics_ that need publishing
   * @param[in] timekeeper The time object to use for header timestamps
   */
  void Publish(const Timekeeper& timekeeper);

  /**
   * @brief Visualize body
   * @param[in] name    The name of the topic
   * @param[in] body The body to output
   * @param[in] r red color 0.0->1.0
   * @param[in] g green color 0.0->1.0
   * @param[in] b blue color 0.0->1.0
   * @param[in] a alpha color 0.0->1.0
   */
  void Visualize(std::string name, b2Body* body, float r, float g, float b,
                 float a);

  /**
   * @brief Visualize body
   * @param[in] name    The name of the topic
   * @param[in] joint The join to output
   * @param[in] r red color 0.0->1.0
   * @param[in] g green color 0.0->1.0
   * @param[in] b blue color 0.0->1.0
   * @param[in] a alpha color 0.0->1.0
   */
  void Visualize(std::string name, b2Joint* joint, float r, float g, float b,
                 float a);

  /**
   * @brief Visualize a layer in 2.5d
   * @param[in] name    The name of the topic
   * @param[in] joint The join to output
   * @param[in] r red color 0.0->1.0
   * @param[in] g green color 0.0->1.0
   * @param[in] b blue color 0.0->1.0
   * @param[in] a alpha color 0.0->1.0
   */
  void VisualizeLayer(std::string name, Body* body);

  /**
   * @brief Remove all elements in a visualization topic
   * @param name
   */
  void Reset(std::string name);

  /**
   * @brief Append body as a marker on the marker array
   * @param[in] markers The output marker array
   * @param[in] body The input body pointer
   * @param[in] r red color 0.0->1.0
   * @param[in] g green color 0.0->1.0
   * @param[in] b blue color 0.0->1.0
   * @param[in] a alpha color 0.0->1.0
   */
  void BodyToMarkers(visualization_msgs::MarkerArray& markers, b2Body* body,
                     float r, float g, float b, float a);

  /**
   * @brief Append a joint as a marker on the marker array
   * @param[in] markers The output marker array
   * @param[in] joint The input joint pointer
   * @param[in] r red color 0.0->1.0
   * @param[in] g green color 0.0->1.0
   * @param[in] b blue color 0.0->1.0
   * @param[in] a alpha color 0.0->1.0
   */
  void JointToMarkers(visualization_msgs::MarkerArray& markers, b2Joint* joint,
                      float r, float g, float b, float a);

  /**
   * @brief Ensure that a topic name is being broadcasted
   * @param[in] name Name of the topic
   */
  void AddTopicIfNotExist(const std::string& name);

  /**
   * @brief Publish topics list
   */
  void PublishTopicList();
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_DEBUG_VISUALIZATION_H
