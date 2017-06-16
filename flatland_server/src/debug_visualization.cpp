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

#include "flatland_server/debug_visualization.h"
#include <Box2D/Box2D.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
 * @param r red color 0.0->1.0
 * @param g green color 0.0->1.0
 * @param b blue color 0.0->1.0
 * @param a alpha color 0.0->1.0
 */
void DebugVisualization::bodyToMarkers(visualization_msgs::MarkerArray& markers,
                                       b2Body* body, float r, float g, float b,
                                       float a) {
  b2Fixture* fixture = body->GetFixtureList();

  while (fixture != NULL) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = markers.markers.size();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.pose.position.x = body->GetPosition().x;
    marker.pose.position.y = body->GetPosition().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, body->GetAngle());  // from euler angles: roll, pitch, yaw
    marker.pose.orientation = tf2::toMsg(q);

    // Get the shape from the fixture
    switch (fixture->GetType()) {
      case b2Shape::e_circle: {
        b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();

        marker.type = marker.CYLINDER;
        marker.scale.x = marker.scale.y = circle->m_radius * 2.0;  // diameter
        marker.scale.z = 0.01;                                     // height

      } break;

      case b2Shape::e_polygon: {  // Convert b2Polygon -> LINE_STRIP
        b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
        marker.type = marker.LINE_STRIP;
        marker.scale.x = 0.03;  // 3cm wide lines

        // TODO: Translate+rotate by body pose
        for (int i = 0; i < poly->m_count; i++) {
          geometry_msgs::Point p;
          p.x = poly->m_vertices[i].x;
          p.y = poly->m_vertices[i].y;
          marker.points.push_back(p);
        }
        marker.points.push_back(marker.points[0]);  // Close the shape

      } break;

      case b2Shape::e_edge: {  // Convert b2Edge -> LINE_LIST
        b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();
        marker.type = marker.LINE_LIST;
        marker.scale.x = 0.03;  // 3cm wide lines

        // TODO: Translate+rotate by body pose
        geometry_msgs::Point p;  // b2Edge uses vertex1 and 2 for its edges
        p.x = edge->m_vertex1.x;
        p.y = edge->m_vertex1.y;
        marker.points.push_back(p);
        p.x = edge->m_vertex2.x;
        p.y = edge->m_vertex2.y;
        marker.points.push_back(p);

      } break;

      default:  // Unsupported shape
        ROS_WARN_THROTTLE_NAMED(1.0, "DebugVis", "Unsupported Box2D shape %d",
                                static_cast<int>(fixture->GetType()));
        break;
    }

    markers.markers.push_back(marker);  // Add the new marker
    fixture = fixture->GetNext();       // Traverse the linked list of fixtures
  }
}

/**
 * @brief Publish all marker array topics that need publishing
 */
void DebugVisualization::publish() {
  // Iterate over the topics map as pair(name, topic)
  for (auto& topic : topics) {
    if (!topic.second.needs_publishing) {
      continue;
    }
    topic.second.publisher.publish(topic.second.markers);
    topic.second.needs_publishing = false;
    ROS_INFO_THROTTLE_NAMED(1.0, "DebugVis", "Publishing %s",
                            topic.first.c_str());
  }
}

/**
 * @brief Append the shapes from a fixture to a marker array
 * @param name    The name of the topic
 * @param fixture The fixture to output
 * @param r red color 0.0->1.0
 * @param g green color 0.0->1.0
 * @param b blue color 0.0->1.0
 * @param a alpha color 0.0->1.0
 */
void DebugVisualization::visualize(std::string name, b2Body* body, float r,
                                   float g, float b, float a) {
  // If the topic doesn't exist, create it
  if (topics.count(name) == 0) {  // If the topic doesn't exist yet, create it
    topics[name] = {
        node.advertise<visualization_msgs::MarkerArray>(name, 1, true), true,
        visualization_msgs::MarkerArray()};
  }

  bodyToMarkers(topics[name].markers, body, r, g, b, a);
  topics[name].needs_publishing = true;
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
