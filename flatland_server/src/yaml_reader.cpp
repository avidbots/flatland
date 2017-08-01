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

#include <flatland_server/exceptions.h>
#include <flatland_server/yaml_reader.h>
#include <boost/algorithm/string.hpp>

namespace flatland_server {

/**
 * @brief Helper function to format the in message adding spaces and brackets
 */
std::string in_fmt(const std::string &msg) {
  if (msg.size() == 0) {
    return "";
  }

  boost::algorithm::to_lower(msg);
  return " (in " + msg + ")";
}

std::string quote(const std::string &msg) { return "\"" + msg + "\""; }

YamlReader::YamlReader(const YAML::Node &node) : node_(node) {}

YamlReader::YamlReader(const std::string &path) {
  try {
    node_ = YAML::LoadFile(quote(path));
  } catch (const YAML::BadFile &e) {
    throw YAMLException("File does not exist, path=" + quote(path), e);

  } catch (const YAML::ParserException &e) {
    throw YAMLException("Malformatted file, path=" + quote(path), e);

  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading file, path=" + quote(path), e);
  }
}

YamlReader YamlReader::Node(const std::string &key, NodeTypeCheck type_check,
                            std::string in = "") {
  if (!node_[key]) {
    throw YAMLException("Missing entry " + quote(key) + " " + err_from_msg);
  }

  if (type_check == NodeTypeCheck::MAP && !node_[key].IsMap()) {
    throw YAMLException("Entry " + quote(key) + " must be a map" + in_fmt(in);

  } else if (type_check == NodeTypeCheck::LIST && !node_[key].IsSequence()) {
    throw YAMLException("Entry " + quote(key) + " must be a list" + in_fmt(in);
  }

  return YamlReader(node_[key]);
}

template <class T>
T YamlReader::Get(const std::string &key, const std::string &type_name,
                  std::string in) {
  if (!node_[key]) {
    throw YAMLException("Missing entry " + quote(key) + prep(in));
  }

  T ret;

  try {
    ret = node_[key].as<T>();
  } catch (const YAML::RepresentationException &e) {
    throw YAMLException("Error converting entry " + quote(key) + " to " +
                        type_name + " " + in_fmt(in));
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error reading entry " + quote(key) + in_fmt(in));
  }

  return ret;
}

template <class T>
T YamlReader::GetOpt(const std::string &key, const T &assume_val,
                     const std::string &type_name, std::string in) {
  if (!node_[key]) {
    return assume_val;
  }

  T ret;

  try {
    ret = node_[key].as<T>();
  } catch (const YAML::RepresentationException &e) {
    throw YAMLException("Error converting entry " + quote(key) + " to " +
                        type_name + " " + in_fmt(in));
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error reading entry " + quote(key) + in_fmt(in));
  }

  return ret;
}

// return asfasfasdfas

// double GetDouble(const std::string &key, const std::string &in = "") {
//   return Get
// }
// std::string GetString();
};
