/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2018 Avidbots Corp.
 * @name	 yaml_preprocessor
 * @brief	 Yaml preprocessor using Lua
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

#include "flatland_server/yaml_preprocessor.h"

#include <ros/ros.h>
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/node/type.h>
#include <yaml-cpp/null.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/lexical_cast.hpp>
#include <cstdlib>
#include <cstring>

#include "flatland_server/exceptions.h"

namespace flatland_server {

static const char *kEvalMarker = "$eval";
static const char *kIncludeMarker = "$include";
static const char *kSequenceIncludeMarker = "$[include]";

void YamlPreprocessor::Parse(YAML::Node &node, const std::string &ref_path) {
  YamlPreprocessor::ProcessNodes(node, ref_path);
}

void YamlPreprocessor::ProcessNodes(YAML::Node &node,
                                    const std::string &ref_path) {
  switch (node.Type()) {
    case YAML::NodeType::Sequence: {
      // copy the elements to a new sequence, checking for $[include] expressions
      // as we go. If an include is found, replace it with one or more nodes
      // parsed from the included file.
      YAML::Node new_sequence = YAML::Node(YAML::NodeType::Sequence);
      for (YAML::Node child : node) {
        std::vector<YAML::Node> included_nodes = {};
        if (ProcessSequenceIncludeNode(included_nodes, child, ref_path)) {
          ROS_INFO_STREAM("Sequence include yielded " << included_nodes.size()
                                                      << " nodes");
          // node was an $include
          for (auto &include_child : included_nodes) {
            // ProcessSequenceIncludeNode handles processing of children itself.
            new_sequence.push_back(include_child);
          }
        } else {
          // not an include, just process and copy over normally.
          // the sequence itself is the parent.
          YamlPreprocessor::ProcessNodes(child, ref_path);
          new_sequence.push_back(child);
        }
      }
      node = new_sequence;
      break;
    }
    case YAML::NodeType::Map:
      for (YAML::iterator it = node.begin(); it != node.end(); ++it) {
        YamlPreprocessor::ProcessNodes(it->second, ref_path);
      }
      break;
    case YAML::NodeType::Scalar: {
      auto s = node.as<std::string>();
      if (s.compare(0, strlen(kEvalMarker), kEvalMarker) == 0) {
        ProcessEvalNode(node);
      } else if (s.compare(0, strlen(kIncludeMarker), kIncludeMarker) == 0) {
        ProcessIncludeNode(node, ref_path);
      }
      break;
    }
    default:
      ROS_DEBUG_STREAM(
          "Yaml Preprocessor found an unexpected type: " << node.Type());
      break;
  }
}

void YamlPreprocessor::ProcessEvalNode(YAML::Node &node) {
  std::string value =
      node.as<std::string>().substr(strlen(kEvalMarker));  // omit the $parse
  boost::algorithm::trim(value);                           // trim whitespace
  ROS_INFO_STREAM("Attempting to parse lua " << value);

  if (value.find("return ") == std::string::npos) {  // Has no return statement
    value = "return " + value;
  }

  // Create the Lua context
  lua_State *L = luaL_newstate();
  luaL_openlibs(L);
  lua_pushcfunction(L, YamlPreprocessor::LuaGetEnv);
  lua_setglobal(L, "env");
  lua_pushcfunction(L, YamlPreprocessor::LuaGetParam);
  lua_setglobal(L, "param");

  try { /* Attempt to run the Lua string and parse its results */
    int error = luaL_dostring(L, value.c_str());
    if (error) {
      ROS_ERROR_STREAM(lua_tostring(L, -1));
      lua_pop(L, 1); /* pop error message from the stack */
    } else {
      int t = lua_type(L, 1);
      if (t == LUA_TNIL) {
        node = "";
        ROS_INFO_STREAM("Preprocessor parsed " << value << " as empty string");
      } else if (t == LUA_TBOOLEAN) {
        ROS_INFO_STREAM("Preprocessor parsed "
                        << value << " as bool "
                        << (lua_toboolean(L, 1) ? "true" : "false"));
        node = lua_toboolean(L, 1) ? "true" : "false";
      } else if (t == LUA_TSTRING || t == LUA_TNUMBER) {
        ROS_INFO_STREAM("Preprocessor parsed " << value << " as "
                                               << lua_tostring(L, 1));
        node = lua_tostring(L, 1);
      } else {
        ROS_ERROR_STREAM("No lua output for " << value);
      }
    }
  } catch (
      ...) { /* Something went wrong parsing the lua, or gettings its results */
    ROS_ERROR_STREAM("Lua error in: " << value);
  }
}
std::string YamlPreprocessor::ResolveIncludeFilePath(
    const std::string &filename, const std::string &ref_path) {
  namespace fs = boost::filesystem;
  fs::path f(filename);
  // already an absolute path, return as-is.
  if (f.is_absolute()) {
    ROS_DEBUG_STREAM("Path is already absolute.");
    return filename;
  }
  // if we're not loading from a file, then we can't resolve relative paths.
  // just pass along and hope the caller is requesting something in the CWD.
  if (ref_path.empty()) {
    ROS_WARN_STREAM(
        "$include specified a relative path but no original filename "
        "specified");
    return filename;
  }

  fs::path rel(ref_path);
  if (fs::is_regular_file(rel)) {
    rel = rel.parent_path();
  }

  fs::path result = rel / f;
  return result.string();
}
void YamlPreprocessor::ProcessIncludeNode(YAML::Node &node,
                                          const std::string &ref_path) {
  // omit the $include
  std::string value = node.as<std::string>().substr(strlen(kIncludeMarker));
  ROS_INFO_STREAM("Attempting to parse include: " << value);
  boost::algorithm::trim(value);  // remove whitespace

  // format the common file & include info for any thrown exceptions.
  const auto format_error_info = [&]() {
    return "path=" + ref_path + ", include=" + value;
  };

  try {
    auto path = ResolveIncludeFilePath(value, ref_path);
    node = YAML::LoadFile(path);
    // recursively process the included file, too
    ProcessNodes(node, path);
    ROS_INFO_STREAM("Successfully loaded include file " + path);
  } catch (const YAML::BadFile &) {
    throw YAMLException("File specified in $include does not exist," +
                        format_error_info());
  } catch (const YAML::ParserException &e) {
    throw YAMLException(
        "Malformatted file specified as include, " + format_error_info(), e);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading include file, " + format_error_info(),
                        e);
  }
}

bool YamlPreprocessor::ProcessSequenceIncludeNode(
    std::vector<YAML::Node> &out_elems, YAML::Node &node,
    const std::string &ref_path) {
  if (node.Type() != YAML::NodeType::Scalar) {
    return false;
  }
  auto node_string = node.as<std::string>();
  // check for the actual sequence include marker
  if (node_string.compare(0, strlen(kSequenceIncludeMarker),
                          kSequenceIncludeMarker) != 0) {
    return false;
  }
  // omit the $include
  std::string value = node_string.substr(strlen(kSequenceIncludeMarker));
  boost::algorithm::trim(value);
  out_elems.clear();

  ROS_INFO_STREAM("Attempting to parse sequence include: " << value);

  // format the common file & include info for any thrown exceptions.
  const auto format_error_info = [&]() {
    return "path=" + ref_path + ", include=" + value;
  };

  try {
    auto path = ResolveIncludeFilePath(value, ref_path);
    out_elems = YAML::LoadAllFromFile(path);

    // recursively process the included nodes, too
    for (auto &included_node : out_elems) {
      ProcessNodes(included_node, path);
    }
    ROS_INFO_STREAM("Successfully loaded sequence include file " + path);

  } catch (const YAML::BadFile &) {
    throw YAMLException("File specified in $[include] does not exist," +
                        format_error_info());
  } catch (const YAML::ParserException &e) {
    throw YAMLException(
        "Malformatted file specified as include, " + format_error_info(), e);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading include file, " + format_error_info(),
                        e);
  }

  // parsed include successfully
  return true;
}

YAML::Node YamlPreprocessor::LoadParse(const std::string &path) {
  YAML::Node node;

  try {
    node = YAML::LoadFile(path);
  } catch (const YAML::BadFile &e) {
    throw YAMLException("File does not exist, path=" + path);
  } catch (const YAML::ParserException &e) {
    throw YAMLException("Malformatted file, path=" + path, e);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading file, path=" + path, e);
  }

  YamlPreprocessor::Parse(node, path);
  return node;
}

int YamlPreprocessor::LuaGetEnv(lua_State *L) {
  const char *name = lua_tostring(L, 1);
  const char *env = std::getenv(name);

  if (lua_gettop(L) == 2 && env == NULL) {  // use default
    if (lua_isnumber(L, 2)) {
      lua_pushnumber(L, lua_tonumber(L, 2));
    } else if (lua_isstring(L, 2)) {
      lua_pushstring(L, lua_tostring(L, 2));
    } else if (lua_isboolean(L, 2)) {
      lua_pushboolean(L, lua_toboolean(L, 2));
    }
  } else {              // no default
    if (env == NULL) {  // Push back a nil
      ROS_WARN_STREAM("No environment variable for: " << name);
      lua_pushnil(L);
    } else {
      ROS_WARN_STREAM("Found env for " << name);
      try {  // Try to push a number
        double x = boost::lexical_cast<double>(env);
        lua_pushnumber(L, x);
      } catch (boost::bad_lexical_cast &) {  // Otherwise it's a string
        lua_pushstring(L, env);
      }
    }
  }

  return 1;  // 1 return value
}

int YamlPreprocessor::LuaGetParam(lua_State *L) {
  const char *name = lua_tostring(L, 1);
  std::string param_s;
  double param_d;
  bool param_b;

  if (lua_gettop(L) == 2 && !ros::param::has(name)) {  // use default
    if (lua_isnumber(L, 2)) {
      lua_pushnumber(L, lua_tonumber(L, 2));
    } else if (lua_isboolean(L, 2)) {
      lua_pushboolean(L, lua_toboolean(L, 2));
    } else if (lua_isstring(L, 2)) {
      lua_pushstring(L, lua_tostring(L, 2));
    } else {
      ROS_WARN_STREAM("Couldn't load int/double/string value at param "
                      << name);
      lua_pushnil(L);
    }
  } else {                         // no default
    if (!ros::param::has(name)) {  // Push back a nil
      ROS_WARN_STREAM("No rosparam found for: " << name);
      lua_pushnil(L);
    } else {
      if (ros::param::get(name, param_d)) {
        lua_pushnumber(L, param_d);
      } else if (ros::param::get(name, param_s)) {
        lua_pushstring(L, param_s.c_str());
      } else if (ros::param::get(name, param_b)) {
        lua_pushstring(L, param_b ? "true" : "false");
      } else {
        ROS_WARN_STREAM("Couldn't load int/double/string value at param "
                        << name);
        lua_pushnil(L);
      }
    }
  }

  return 1;  // 1 return value
}
}  // namespace flatland_server