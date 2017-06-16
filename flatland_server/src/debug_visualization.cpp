/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   debug_visualization.cpp
 * @brief  Transform box2d types into published visualization messages
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

#include <Box2D/Box2D.h>
#include <ros/ros.h>
#include <map>
#include <string>

namespace flatland_server {

DebugVisualization::DebugVisualization() : node("~debug") {}

/**
 * @brief Return the singleton object
 */
DebugVisualization& DebugVisualization::get() {
  static DebugVisualization instance;
  return instance;
}

/**
 * @brief Append each shape on the fixture as a marker on the marker array
 * @param markers The output marker array
 * @param fixture The input fixture pointer
 */
void DebugVisualization::fixtureToMarkers(
    visualization_msgs::MarkerArray& markers, b2Fixture* fixture) {}

/**
 * @brief Publish all marker array topics that need publishing
 */
void DebugVisualization::publish() {
  for (auto topic : topics) {
    if (!topic.needs_publishing) {
      continue;
    }
    topic.publisher.publish(topic.markers);
    topic.needs_publish = false;
  }
}

/**
 * @brief Append the shapes from a fixture to a marker array
 * @param name    The name of the topic
 * @param fixture The fixture to output
 */
void DebugVisualization::visualize(std::string name, b2Fixture* fixture) {
  // If the topic doesn't exist, create it
  if (topics.count(name) == 0) {  // If the topic doesn't exist yet, create it
    topics[name] = {

        n.advertise<visualization_msgs::MarkerArray>(name, 1000) true,
        visualization_msgs::MarkerArray()};
  }

  // Todo: Actually do things!
}

/**
 * @brief Remove all elements in a visualiation topic
 * @param name
 */
void DebugVisualization::reset(std::string name) {
  if (topics.count(name) > 0) {  // If the topic exists, clear it
    topics[name].markers.markers.clear();
    topics[name].needs_publishing = true;
  }
}

};  // namespace flatland_server
