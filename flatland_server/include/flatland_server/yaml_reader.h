/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 yaml_reader.h
 * @brief	 Defines yaml_reader
 * @author   Chunshang Li
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

#ifndef FLATLAND_SERVER_YAML_READER_H
#define FLATLAND_SERVER_YAML_READER_H

#include <Box2D/Box2D.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/types.h>
#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string.hpp>
#include <boost/type_index.hpp>
#include <string>
#include <vector>

namespace flatland_server {

/**
 */
class YamlReader {
 public:
  enum NodeTypeCheck { MAP, LIST, NO_CHECK };

  YAML::Node node_;                           ///< The YAML Node this processes
  std::map<std::string, bool> key_accessed_;  /// Records of the keys processed
  std::string error_location_msg_;
  std::string in_;

  /**
   * @brief
   */
  YamlReader();
  YamlReader(const YAML::Node &node, std::string error_location_msg = "");
  YamlReader(const std::string &path, std::string error_location_msg = "");

  void SetErrorLocationMsg(const std::string &msg);

  YAML::Node Node();

  bool IsNodeNull();
  int NodeSize();

  YamlReader SubNode(int index, NodeTypeCheck type_check,
                     std::string error_location_msg = "");

  YamlReader SubNode(const std::string &key, NodeTypeCheck type_check,
                     std::string error_location_msg = "");

  YamlReader SubNodeOpt(const std::string &key, NodeTypeCheck type_check,
                        std::string error_location_msg = "");

  template <typename T>
  T Get(const std::string &key);

  template <typename T>
  T Get(const std::string &key, const T &default_val);

  template <typename T>
  std::vector<T> GetList(const std::string &key, int min_size, int max_size);

  template <typename T>
  std::vector<T> GetList(const std::string &key,
                         const std::vector<T> default_val, int min_size,
                         int max_size);

  Vec2 GetVec2(const std::string &key);

  Vec2 GetVec2(const std::string &key, const Vec2 &default_val);

  Color GetColor(const std::string &key, const Color &default_val);

  Pose GetPose(const std::string &key);

  Pose GetPose(const std::string &key, const Pose &default_val);
};

inline std::string Q(const std::string &msg) { return "\"" + msg + "\""; }

template <typename T>
T YamlReader::Get(const std::string &key) {
  if (!node_[key]) {
    throw YAMLException("Entry key=" + Q(key) + " does not exist" + in_);
  }

  T ret;

  try {
    ret = node_[key].as<T>();
  } catch (const YAML::RepresentationException &e) {
    throw YAMLException("Error converting entry key=" + Q(key) + " to " +
                        boost::typeindex::type_id<T>().pretty_name() +
                        in_);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error reading entry key=" + Q(key) + in_);
  }

  return ret;
}

template <typename T>
T YamlReader::Get(const std::string &key, const T &default_val) {
  if (!node_[key]) {
    return default_val;
  }

  return Get<T>(key);
}

template <typename T>
std::vector<T> YamlReader::GetList(const std::string &key, int min_size,
                                   int max_size) {
  if (!node_[key]) {
    throw YAMLException("Entry key=" + Q(key) + " does not exist" + in_);
  }

  YamlReader reader = SubNode(key, LIST);
  YAML::Node n = reader.Node();

  std::vector<T> list;

  if (min_size > 0 && max_size > 0 && min_size == max_size &&
      n.size() != max_size) {
    throw YAMLException("Entry " + Q(key) + " must have size of exactly " +
                        std::to_string(min_size) + in_);
  }

  if (min_size > 0 && n.size() < min_size) {
    throw YAMLException("Entry " + Q(key) + " must have size <" +
                        std::to_string(min_size) + in_);
  }

  if (max_size > 0 && n.size() > max_size) {
    throw YAMLException("Entry " + Q(key) + " must have size >" +
                        std::to_string(max_size) + in_);
  }

  for (int i = 0; i < n.size(); i++) {
    T val;
    try {
      val = n[i].as<T>();
    } catch (const YAML::RepresentationException &e) {
      throw YAMLException(
          "Error converting entry index=" + std::to_string(i) + " to " +
          boost::typeindex::type_id<T>().pretty_name() + in_);
    } catch (const YAML::Exception &e) {
      throw YAMLException("Error reading entry index=" + std::to_string(i) +
                          in_);
    }

    list.push_back(val);
  }

  return list;
}

template <typename T>
std::vector<T> YamlReader::GetList(const std::string &key,
                                   const std::vector<T> default_val,
                                   int min_size, int max_size) {
  if (!node_[key]) {
    return default_val;
  }

  return GetList<T>(key, min_size, max_size);
}
}

// encode and decode functions for yaml-cpp to convert values for commonly used
// types in flatland server
namespace YAML {
template <>
struct convert<b2Vec2> {
  static bool decode(const Node &node, b2Vec2 &rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    return true;
  }
};

template <>
struct convert<flatland_server::Vec2> {
  static bool decode(const Node &node, flatland_server::Vec2 &rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    return true;
  }
};

template <>
struct convert<flatland_server::Color> {
  static bool decode(const Node &node, flatland_server::Color &rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.r = node[0].as<double>();
    rhs.g = node[1].as<double>();
    rhs.b = node[2].as<double>();
    rhs.a = node[3].as<double>();
    return true;
  }
};

template <>
struct convert<flatland_server::Pose> {
  static bool decode(const Node &node, flatland_server::Pose &rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.theta = node[2].as<double>();
    return true;
  }
};
}

#endif
