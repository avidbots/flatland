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

#include <flatland_server/yaml_reader.h>
#include <boost/algorithm/string.hpp>

namespace flatland_server {

/**
 * @brief Helper function to format the in message adding spaces and brackets
 */
std::string YamlReader::in_fmt(const std::string &msg) {
  if (msg.size() == 0) {
    return "";
  }
  std::string msg_cpy = msg;
  boost::algorithm::to_lower(msg_cpy);
  return " (in " + msg_cpy + ")";
}

std::string YamlReader::quote(const std::string &msg) { return "\"" + msg + "\""; }

YamlReader::YamlReader(const YAML::Node &node) : node_(node) {}

YamlReader::YamlReader(const std::string &path) {
  try {
    node_ = YAML::LoadFile(path);
  } catch (const YAML::BadFile &e) {
    throw YAMLException("File does not exist, path=" + quote(path), e);

  } catch (const YAML::ParserException &e) {
    throw YAMLException("Malformatted file, path=" + quote(path), e);

  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading file, path=" + quote(path), e);
  }
}

YAML::Node YamlReader::Node() { return node_; }

bool YamlReader::IsNodeNull() { return node_.IsNull(); }

int YamlReader::NodeSize() { return node_.size(); }

YamlReader YamlReader::SubNode(int index, NodeTypeCheck type_check,
                               std::string in) {
  if (!node_[index]) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  does not exist" + in_fmt(in));
  }

  if (type_check == NodeTypeCheck::MAP && !node_[index].IsMap()) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  must be a map" + in_fmt(in));

  } else if (type_check == NodeTypeCheck::LIST && !node_[index].IsSequence()) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  must be a list" + in_fmt(in));
  }

  return YamlReader(node_[index]);
}

YamlReader YamlReader::SubNode(const std::string &key, NodeTypeCheck type_check,
                               std::string in) {
  if (!node_[key]) {
    throw YAMLException("Entry key=" + quote(key) + " does not exist" +
                        in_fmt(in));
  }

  if (type_check == NodeTypeCheck::MAP && !node_[key].IsMap()) {
    throw YAMLException("Entry key=" + quote(key) + " must be a map" +
                        in_fmt(in));

  } else if (type_check == NodeTypeCheck::LIST && !node_[key].IsSequence()) {
    throw YAMLException("Entry key=" + quote(key) + " must be a list" +
                        in_fmt(in));
  }

  return YamlReader(node_[key]);
}

YamlReader YamlReader::SubNodeOpt(const std::string &key,
                                  NodeTypeCheck type_check, std::string in) {
  if (!node_[key]) {
    return YamlReader(YAML::Node());
  }
  return SubNode(key, type_check, in);
}

Vec2 YamlReader::GetVec2(const std::string &key, std::string in) {
  std::vector<double> v = GetList<double>(key, 2, 2, in);
  return Vec2(v[0], v[1]);
}

Color YamlReader::GetColorOpt(const std::string &key, const Color &default_val,
                              std::string in) {
  if (!node_[key]) {
    return default_val;
  }

  if (!node_[key].IsSequence() || node_[key].size() != 4) {
    throw YAMLException("Color entry " + quote(key) +
                        " must be a list of exactly 4 numbers" + in_fmt(in));
  }

  Color c;

  try {
    c.r = node_[key][0].as<double>();
    c.g = node_[key][1].as<double>();
    c.b = node_[key][2].as<double>();
    c.a = node_[key][2].as<double>();
  } catch (const YAML::RepresentationException &e) {
    throw YAMLException("Error converting entry key=" + quote(key) +
                        " to floats " + in_fmt(in));
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error reading entry key=" + quote(key) + in_fmt(in));
  }

  return c;
}

Pose YamlReader::GetPose(const std::string &key, std::string in) {
  if (!node_[key]) {
    throw YAMLException("Entry key=" + quote(key) + " does not exist" +
                        in_fmt(in));
  }

  if (!node_[key].IsSequence() || node_[key].size() != 3) {
    throw YAMLException("Pose entry " + quote(key) +
                        " must be a list of exactly 3 numbers" + in_fmt(in));
  }

  Pose p;

  try {
    p.x = node_[key][0].as<double>();
    p.y = node_[key][1].as<double>();
    p.theta = node_[key][2].as<double>();
  } catch (const YAML::RepresentationException &e) {
    throw YAMLException("Error converting entry key=" + quote(key) +
                        " to floats " + in_fmt(in));
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error reading entry key=" + quote(key) + in_fmt(in));
  }

  return p;
}

Pose YamlReader::GetPoseOpt(const std::string &key, const Pose &default_val,
                            std::string in) {
  if (!node_[key]) {
    return default_val;
  }

  return GetPose(key, in);
}
};
