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

#include <flatland_server/exceptions.h>
#include <flatland_server/types.h>
#include <yaml-cpp/yaml.h>
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

  /**
   * @brief
   */
  YamlReader(const YAML::Node &node);
  YamlReader(const std::string &path);

  YAML::Node Node();

  bool IsNodeNull();
  int NodeSize();

  YamlReader SubNode(int index, NodeTypeCheck type_check, std::string in = "");
  YamlReader SubNode(const std::string &key, NodeTypeCheck type_check,
                     std::string in = "");
  YamlReader SubNodeOpt(const std::string &key, NodeTypeCheck type_check,
                        std::string in = "");

  template <typename T>
  T Get(const std::string &key, std::string in = "");

  template <typename T>
  T GetOpt(const std::string &key, const T &default_val, std::string in = "");

  template <typename T>
  std::vector<T> GetList(const std::string &key, int min_size, int max_size,
                         std::string in = "");

  template <typename T>
  std::vector<T> GetListOpt(const std::string &key,
                            const std::vector<T> default_val, int min_size,
                            int max_size, std::string in = "");

  Vec2 GetVec2(const std::string &key, std::string in = "");
  Color GetColorOpt(const std::string &key, const Color &default_val,
                    std::string in = "");

  Pose GetPose(const std::string &key, std::string in = "");
  Pose GetPoseOpt(const std::string &key, const Pose &default_val,
                  std::string in = "");

private:
  std::string in_fmt(const std::string &msg);
  std::string quote(const std::string &msg);
};

template <typename T>
T YamlReader::Get(const std::string &key, std::string in) {
  if (!node_[key]) {
    throw YAMLException("Entry key=" + quote(key) + " does not exist" +
                        in_fmt(in));
  }

  T ret;

  try {
    ret = node_[key].as<T>();
  } catch (const YAML::RepresentationException &e) {
    throw YAMLException("Error converting entry key=" + quote(key) + " to " +
                        boost::typeindex::type_id<T>().pretty_name() + " " +
                        in_fmt(in));
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error reading entry key=" + quote(key) + in_fmt(in));
  }

  return ret;
}

template <typename T>
T YamlReader::GetOpt(const std::string &key, const T &default_val,
                     std::string in) {
  if (!node_[key]) {
    return default_val;
  }

  return Get<T>(key, in);
}

template <typename T>
std::vector<T> YamlReader::GetList(const std::string &key, int min_size,
                                   int max_size, std::string in) {
  if (!node_[key]) {
    throw YAMLException("Entry key=" + quote(key) + " does not exist" +
                        in_fmt(in));
  }

  YamlReader yr = SubNode(key, LIST, in);
  YAML::Node n = yr.Node();

  std::vector<T> list;

  if (min_size > 0 && max_size > 0 && min_size == max_size &&
      n.size() != max_size) {
    throw YAMLException("Entry " + quote(key) + " must have size of exactly " +
                        std::to_string(min_size) + in_fmt(in));
  }

  if (min_size > 0 && n.size() < min_size) {
    throw YAMLException("Entry " + quote(key) + " must have size <" +
                        std::to_string(min_size) + in_fmt(in));
  }

  if (max_size > 0 && n.size() > max_size) {
    throw YAMLException("Entry " + quote(key) + " must have size >" +
                        std::to_string(max_size) + in_fmt(in));
  }

  for (int i = 0; i < n.size(); i++) {
    T val;
    try {
      val = n[i].as<T>();
    } catch (const YAML::RepresentationException &e) {
      throw YAMLException(
          "Error converting entry index=" + std::to_string(i) + " to " +
          boost::typeindex::type_id<T>().pretty_name() + " " + in_fmt(in));
    } catch (const YAML::Exception &e) {
      throw YAMLException("Error reading entry index=" + std::to_string(i) +
                          in_fmt(in));
    }

    list.push_back(val);
  }
}

template <typename T>
std::vector<T> YamlReader::GetListOpt(const std::string &key,
                                      const std::vector<T> default_val,
                                      int min_size, int max_size,
                                      std::string in) {
  if (!node_[key]) {
    return default_val;
  }

  GetList<T>(key, min_size, max_size, in);
}
};

#endif
