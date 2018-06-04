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

#include <flatland_server/yaml_preprocessor.h>
#include <flatland_server/yaml_reader.h>
#include <boost/filesystem/path.hpp>

namespace flatland_server {

YamlReader::YamlReader() : node_(YAML::Node()) {
  SetErrorInfo("_NONE_", "_NONE_");
}

YamlReader::YamlReader(const YAML::Node &node) : node_(node) {
  YamlPreprocessor::Parse(node_);
  SetErrorInfo("_NONE_", "_NONE_");
}

YamlReader::YamlReader(const std::string &path) {
  try {
    node_ = YAML::LoadFile(path);
    YamlPreprocessor::Parse(node_);
  } catch (const YAML::BadFile &e) {
    throw YAMLException("File does not exist, path=" + Q(path));

  } catch (const YAML::ParserException &e) {
    throw YAMLException("Malformatted file, path=" + Q(path), e);

  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading file, path=" + Q(path), e);
  }

  SetErrorInfo("_NONE_", "_NONE_");
  SetFile(path);
}

YAML::Node YamlReader::Node() { return node_; }

bool YamlReader::IsNodeNull() { return node_.IsNull(); }

int YamlReader::NodeSize() { return node_.size(); }

YamlReader YamlReader::Subnode(int index, NodeTypeCheck type_check,
                               std::string subnode_location) {
  YamlReader reader(node_[index]);

  // default to use the error location message of its parent
  std::string location = subnode_location == ""
                             ? (entry_location_ + " " + entry_name_)
                             : subnode_location;
  reader.SetErrorInfo(location, "index=" + std::to_string(index));
  reader.SetFile(file_path_);

  if (reader.IsNodeNull()) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  does not exist" + reader.fmt_in_);
  } else if (type_check == NodeTypeCheck::MAP && !node_[index].IsMap()) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  must be a map" + reader.fmt_in_);

  } else if (type_check == NodeTypeCheck::LIST && !node_[index].IsSequence()) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  must be a list" + reader.fmt_in_);
  }

  return reader;
}

YamlReader YamlReader::Subnode(const std::string &key, NodeTypeCheck type_check,
                               std::string subnode_location) {
  accessed_keys_.insert(key);
  YamlReader reader(node_[key]);

  // default to use the error location message of its parent
  std::string location = subnode_location == ""
                             ? (entry_location_ + " " + entry_name_)
                             : subnode_location;
  reader.SetErrorInfo(location, Q(key));
  reader.SetFile(file_path_);

  if (!node_[key]) {
    throw YAMLException("Entry " + Q(key) + " does not exist" + reader.fmt_in_);
  } else if (type_check == NodeTypeCheck::MAP && !node_[key].IsMap()) {
    throw YAMLException("Entry " + Q(key) + " must be a map" + reader.fmt_in_);
  } else if (type_check == NodeTypeCheck::LIST && !node_[key].IsSequence()) {
    throw YAMLException("Entry " + Q(key) + " must be a list" + reader.fmt_in_);
  }

  return reader;
}

YamlReader YamlReader::SubnodeOpt(const std::string &key,
                                  NodeTypeCheck type_check,
                                  std::string subnode_location) {
  if (!node_[key]) {
    accessed_keys_.insert(key);
    return YamlReader(YAML::Node());
  }
  return Subnode(key, type_check, subnode_location);
}

Vec2 YamlReader::GetVec2(const std::string &key) {
  std::array<double, 2> v = GetArray<double, 2>(key);
  return Vec2(v[0], v[1]);
}

Vec2 YamlReader::GetVec2(const std::string &key, const Vec2 &vec2) {
  if (!node_[key]) {
    accessed_keys_.insert(key);
    return vec2;
  }

  return GetVec2(key);
}

Color YamlReader::GetColor(const std::string &key, const Color &default_val) {
  std::array<double, 4> v = GetArray<double, 4>(
      key, {default_val.r, default_val.g, default_val.b, default_val.a});
  return Color(v[0], v[1], v[2], v[3]);
}

Pose YamlReader::GetPose(const std::string &key) {
  std::array<double, 3> v = GetArray<double, 3>(key);
  return Pose(v[0], v[1], v[2]);
}

Pose YamlReader::GetPose(const std::string &key, const Pose &default_val) {
  if (!node_[key]) {
    accessed_keys_.insert(key);
    return default_val;
  }

  return GetPose(key);
}

void YamlReader::SetErrorInfo(std::string entry_location,
                              std::string entry_name) {
  boost::algorithm::trim(entry_location);
  boost::algorithm::trim(entry_name);

  if (entry_location == "_NONE_") {
    entry_location_ = "";
  } else if (entry_location != "") {
    entry_location_ = entry_location;
  }

  if (entry_name == "_NONE_") {
    entry_name_ = "";
  } else if (entry_name != "") {
    entry_name_ = entry_name;
  }

  if (entry_location_.size() == 0) {
    fmt_in_ = "";
  } else {
    std::string msg = entry_location_;
    boost::algorithm::to_lower(msg);
    fmt_in_ = " (in " + msg + ")";
  }

  if (entry_name_.size() == 0) {
    fmt_name_ = "";
  } else {
    std::string msg = entry_name_;
    boost::algorithm::to_lower(msg);
    fmt_name_ = " " + msg;
  }
}

void YamlReader::SetFile(const std::string &file_path) {
  if (file_path == "_NONE_") {
    file_path_ = "";
  } else {
    file_path_ = file_path;
  }

  filename_ = boost::filesystem::path(file_path).filename().string();
}

void YamlReader::EnsureAccessedAllKeys() {
  if (!node_.IsMap()) {
    throw YAMLException("Entry" + fmt_name_ + " should be a map" + fmt_in_);
  }

  std::vector<std::string> unused_keys;
  std::vector<std::string> keys;

  for (const auto &k : node_) {
    keys.push_back(k.first.as<std::string>());
  }

  std::sort(keys.begin(), keys.end());

  for (const auto &key : keys) {
    if (accessed_keys_.count(key) == 0) {
      unused_keys.push_back("\"" + key + "\"");
    }
  }

  std::string keys_str = "{" + boost::algorithm::join(keys, ", ") + "}";
  std::string unused_keys_str =
      "{" + boost::algorithm::join(unused_keys, ", ") + "}";

  if (unused_keys.size() > 0) {
    throw YAMLException("Entry" + fmt_name_ +
                        " contains unrecognized entry(s) " + unused_keys_str +
                        fmt_in_);
  }
}
}