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
#include <ros/master.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <map>
#include <string>

namespace flatland_server {

DebugVisualization::DebugVisualization() : node_("~debug") {
  topic_list_publisher_ =
      node_.advertise<flatland_msgs::DebugTopicList>("topics", 0, true);
}

DebugVisualization& DebugVisualization::Get() {
  static DebugVisualization instance;
  return instance;
}

void DebugVisualization::JointToMarkers(
    visualization_msgs::MarkerArray& markers, b2Joint* joint, float r, float g,
    float b, float a) {
  if (joint->GetType() == e_distanceJoint ||
      joint->GetType() == e_pulleyJoint || joint->GetType() == e_mouseJoint) {
    ROS_ERROR_NAMED("DebugVis",
                    "Unimplemented visualization joints. See b2World.cpp for "
                    "implementation");
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.type = marker.LINE_LIST;
  marker.scale.x = 0.01;

  geometry_msgs::Point p_a1, p_a2, p_b1, p_b2;
  p_a1.x = joint->GetAnchorA().x;
  p_a1.y = joint->GetAnchorA().y;
  p_a2.x = joint->GetAnchorB().x;
  p_a2.y = joint->GetAnchorB().y;
  p_b1.x = joint->GetBodyA()->GetPosition().x;
  p_b1.y = joint->GetBodyA()->GetPosition().y;
  p_b2.x = joint->GetBodyB()->GetPosition().x;
  p_b2.y = joint->GetBodyB()->GetPosition().y;

  // Visualization shows lines from bodyA to anchorA, bodyB to anchorB, and
  // anchorA to anchorB
  marker.id = markers.markers.size();
  marker.points.push_back(p_b1);
  marker.points.push_back(p_a1);
  marker.points.push_back(p_b2);
  marker.points.push_back(p_a2);
  marker.points.push_back(p_a1);
  marker.points.push_back(p_a2);

  markers.markers.push_back(marker);

  marker.id = markers.markers.size();
  marker.type = marker.CUBE_LIST;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.03;
  marker.points.clear();
  marker.points.push_back(p_a1);
  marker.points.push_back(p_a2);
  marker.points.push_back(p_b1);
  marker.points.push_back(p_b2);
  markers.markers.push_back(marker);
}

void DebugVisualization::BodyToMarkers(visualization_msgs::MarkerArray& markers,
                                       b2Body* body, float r, float g, float b,
                                       float a) {
  b2Fixture* fixture = body->GetFixtureList();

  while (fixture != NULL) {  // traverse fixture linked list
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.id = markers.markers.size();
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.pose.position.x = body->GetPosition().x;
    marker.pose.position.y = body->GetPosition().y;
    tf2::Quaternion q;  // use tf2 to convert 2d yaw -> 3d quaternion
    q.setRPY(0, 0, body->GetAngle());  // from euler angles: roll, pitch, yaw
    marker.pose.orientation = tf2::toMsg(q);
    bool add_marker = true;

    // Get the shape from the fixture
    switch (fixture->GetType()) {
      case b2Shape::e_circle: {
        b2CircleShape* circle = (b2CircleShape*)fixture->GetShape();

        marker.type = marker.SPHERE_LIST;
        float diameter = circle->m_radius * 2.0;
        marker.scale.z = 0.01;
        marker.scale.x = diameter;
        marker.scale.y = diameter;

        geometry_msgs::Point p;
        p.x = circle->m_p.x;
        p.y = circle->m_p.y;
        marker.points.push_back(p);

      } break;

      case b2Shape::e_polygon: {  // Convert b2Polygon -> LINE_STRIP
        b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
        marker.type = marker.LINE_STRIP;
        marker.scale.x = 0.03;  // 3cm wide lines

        for (int i = 0; i < poly->m_count; i++) {
          geometry_msgs::Point p;
          p.x = poly->m_vertices[i].x;
          p.y = poly->m_vertices[i].y;
          marker.points.push_back(p);
        }
        marker.points.push_back(marker.points[0]);  // Close the shape

      } break;

      case b2Shape::e_edge: {    // Convert b2Edge -> LINE_LIST
        geometry_msgs::Point p;  // b2Edge uses vertex1 and 2 for its edges
        b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();

        // If the last marker is a line list, extend it
        if (markers.markers.size() > 0 &&
            markers.markers.back().type == marker.LINE_LIST) {
          add_marker = false;
          p.x = edge->m_vertex1.x;
          p.y = edge->m_vertex1.y;
          markers.markers.back().points.push_back(p);
          p.x = edge->m_vertex2.x;
          p.y = edge->m_vertex2.y;
          markers.markers.back().points.push_back(p);

        } else {  // otherwise create a new line list

          marker.type = marker.LINE_LIST;
          marker.scale.x = 0.03;  // 3cm wide lines

          p.x = edge->m_vertex1.x;
          p.y = edge->m_vertex1.y;
          marker.points.push_back(p);
          p.x = edge->m_vertex2.x;
          p.y = edge->m_vertex2.y;
          marker.points.push_back(p);
        }

      } break;

      default:  // Unsupported shape
        ROS_WARN_THROTTLE_NAMED(1.0, "DebugVis", "Unsupported Box2D shape %d",
                                static_cast<int>(fixture->GetType()));
        fixture = fixture->GetNext();
        continue;  // Do not add broken marker
        break;
    }

    if (add_marker) {
      markers.markers.push_back(marker);  // Add the new marker
    }
    fixture = fixture->GetNext();  // Traverse the linked list of fixtures
  }
}

void DebugVisualization::Publish(const Timekeeper& timekeeper) {
  // Iterate over the topics_ map as pair(name, topic)

  std::vector<std::string> to_delete;

  for (auto& topic : topics_) {
    if (!topic.second.needs_publishing) {
      continue;
    }

    // since if empty markers are published rviz will continue to publish
    // using the old data, delete the topic list
    if (topic.second.markers.markers.size() == 0) {
      to_delete.push_back(topic.first);
    } else {
      // Iterate the marker array to update all the timestamps
      for (unsigned int i = 0; i < topic.second.markers.markers.size(); i++) {
        topic.second.markers.markers[i].header.stamp = timekeeper.GetSimTime();
      }
      topic.second.publisher.publish(topic.second.markers);
      topic.second.needs_publishing = false;
    }
  }

  if (to_delete.size() > 0) {
    for (const auto& topic : to_delete) {
      ROS_WARN_NAMED("DebugVis", "Deleting topic %s", topic.c_str());
      topics_.erase(topic);
    }
    PublishTopicList();
  }
}

void DebugVisualization::VisualizeLayer(std::string name, Body* body) {
  AddTopicIfNotExist(name);

  b2Fixture* fixture = body->physics_body_->GetFixtureList();

  visualization_msgs::Marker marker;
  if (fixture == NULL) return;  // Nothing to visualize, empty linked list

  while (fixture != NULL) {  // traverse fixture linked list

    marker.header.frame_id = "map";
    marker.id = topics_[name].markers.markers.size();
    marker.color.r = body->color_.r;
    marker.color.g = body->color_.g;
    marker.color.b = body->color_.b;
    marker.color.a = body->color_.a;
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.frame_locked = true;
    marker.pose.position.x = body->physics_body_->GetPosition().x;
    marker.pose.position.y = body->physics_body_->GetPosition().y;

    tf2::Quaternion q;  // use tf2 to convert 2d yaw -> 3d quaternion
    q.setRPY(0, 0, body->physics_body_
                       ->GetAngle());  // from euler angles: roll, pitch, yaw
    marker.pose.orientation = tf2::toMsg(q);
    marker.type = marker.TRIANGLE_LIST;

    YamlReader reader(body->properties_);
    YamlReader debug_reader =
        reader.SubnodeOpt("debug", YamlReader::NodeTypeCheck::MAP);
    float min_z = debug_reader.Get<float>("min_z", 0.0);
    float max_z = debug_reader.Get<float>("max_z", 1.0);

    // Get the shape from the fixture
    if (fixture->GetType() == b2Shape::e_edge) {
      geometry_msgs::Point p;  // b2Edge uses vertex1 and 2 for its edges
      b2EdgeShape* edge = (b2EdgeShape*)fixture->GetShape();

      p.x = edge->m_vertex1.x;
      p.y = edge->m_vertex1.y;
      p.z = min_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex2.x;
      p.y = edge->m_vertex2.y;
      p.z = min_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex2.x;
      p.y = edge->m_vertex2.y;
      p.z = max_z;
      marker.points.push_back(p);

      p.x = edge->m_vertex1.x;
      p.y = edge->m_vertex1.y;
      p.z = min_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex2.x;
      p.y = edge->m_vertex2.y;
      p.z = max_z;
      marker.points.push_back(p);
      p.x = edge->m_vertex1.x;
      p.y = edge->m_vertex1.y;
      p.z = max_z;
      marker.points.push_back(p);
    }

    fixture = fixture->GetNext();  // Traverse the linked list of fixtures
  }

  topics_[name].markers.markers.push_back(marker);  // Add the new marker
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Visualize(std::string name, b2Body* body, float r,
                                   float g, float b, float a) {
  AddTopicIfNotExist(name);
  BodyToMarkers(topics_[name].markers, body, r, g, b, a);
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Visualize(std::string name, b2Joint* joint, float r,
                                   float g, float b, float a) {
  AddTopicIfNotExist(name);
  JointToMarkers(topics_[name].markers, joint, r, g, b, a);
  topics_[name].needs_publishing = true;
}

void DebugVisualization::Reset(std::string name) {
  if (topics_.count(name) > 0) {  // If the topic exists, clear it
    topics_[name].markers.markers.clear();
    topics_[name].needs_publishing = true;
  }
}

void DebugVisualization::AddTopicIfNotExist(const std::string& name) {
  // If the topic doesn't exist yet, create it
  if (topics_.count(name) == 0) {
    topics_[name] = {
        node_.advertise<visualization_msgs::MarkerArray>(name, 0, true), true,
        visualization_msgs::MarkerArray()};

    ROS_INFO_ONCE_NAMED("DebugVis", "Visualizing %s", name.c_str());
    PublishTopicList();
  }
}

void DebugVisualization::PublishTopicList() {
  flatland_msgs::DebugTopicList topic_list;
  for (auto const& topic_pair : topics_)
    topic_list.topics.push_back(topic_pair.first);
  topic_list_publisher_.publish(topic_list);
}
};  // namespace flatland_server
