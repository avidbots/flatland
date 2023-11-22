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

#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace flatland_server
{

void YamlPreprocessor::Parse(YAML::Node & node) { this->ProcessNodes(node); }

void YamlPreprocessor::ProcessNodes(YAML::Node & node)
{
  switch (node.Type()) {
    case YAML::NodeType::Sequence:
      for (YAML::Node child : node) {
        this->ProcessNodes(child);
      }
      break;
    case YAML::NodeType::Map:
      for (YAML::iterator it = node.begin(); it != node.end(); ++it) {
        this->ProcessNodes(it->second);
      }
      break;
    case YAML::NodeType::Scalar:
      if (node.as<std::string>().compare(0, 5, "$eval") == 0) {
        this->ProcessScalarNode(node);
      }
      break;
    default:
      RCLCPP_DEBUG_STREAM(
        rclcpp::get_logger("Yaml Preprocessor"),
        "Yaml Preprocessor found an unexpected type: " << node.Type());
      break;
  }
}

void YamlPreprocessor::ProcessScalarNode(YAML::Node & node)
{
  std::string value = node.as<std::string>().substr(5);  // omit the $eval
  boost::algorithm::trim(value);                         // trim whitespace
  RCLCPP_INFO_STREAM(rclcpp::get_logger("YAML Preprocessor"), "Attempting to parse lua " << value);

  if (value.find("return ") == std::string::npos) {  // Has no return statement
    value = "return " + value;
  }

  // Create the Lua context
  lua_State * L = luaL_newstate();
  luaL_openlibs(L);
  lua_pushcfunction(L, YamlPreprocessor::LuaGetEnv);
  lua_setglobal(L, "env");
  lua_pushcfunction(L, YamlPreprocessor::LuaGetParam);
  lua_setglobal(L, "param");
  lua_pushlightuserdata(L, (void *)this);
  lua_setglobal(L, "class_pointer");

  try { /* Attempt to run the Lua string and parse its results */
    int error = luaL_dostring(L, value.c_str());
    if (error) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("Yaml Preprocessor"), lua_tostring(L, -1));
      lua_pop(L, 1); /* pop error message from the stack */
    } else {
      int t = lua_type(L, 1);
      if (t == LUA_TNIL) {
        node = "";
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("YAML Preprocessor"),
          "Preprocessor parsed " << value << " as empty string");
      } else if (t == LUA_TBOOLEAN) {
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("YAML Preprocessor"), "Preprocessor parsed "
                                                     << value << " as bool "
                                                     << (lua_toboolean(L, 1) ? "true" : "false"));
        node = lua_toboolean(L, 1) ? "true" : "false";
      } else if (t == LUA_TSTRING || t == LUA_TNUMBER) {
        RCLCPP_INFO_STREAM(
          rclcpp::get_logger("YAML Preprocessor"),
          "Preprocessor parsed " << value << " as " << lua_tostring(L, 1));
        node = lua_tostring(L, 1);
      } else {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("Yaml Preprocessor"), "No lua output for " << value);
      }
    }
  } catch (...) {
    /* Something went wrong parsing the lua, or gettings its results */
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("Yaml Preprocessor"), "Lua error in: " << value);
  }
}

YAML::Node YamlPreprocessor::LoadParse(const std::string & path)
{
  YAML::Node node;

  try {
    node = YAML::LoadFile(path);
  } catch (const YAML::BadFile & e) {
    throw YAMLException("File does not exist, path=" + path);
  } catch (const YAML::ParserException & e) {
    throw YAMLException("Malformatted file, path=" + path, e);
  } catch (const YAML::Exception & e) {
    throw YAMLException("Error loading file, path=" + path, e);
  }

  this->Parse(node);
  return node;
}

int YamlPreprocessor::LuaGetEnv(lua_State * L)
{
  const char * name = lua_tostring(L, 1);
  const char * env = std::getenv(name);

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
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("Yaml Preprocessor"), "No environment variable for: " << name);
      lua_pushnil(L);
    } else {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("Yaml Preprocessor"), "Found env for " << name);
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

int YamlPreprocessor::LuaGetParam(lua_State * L)
{
  const char * name = lua_tostring(L, 1);
  rclcpp::Parameter param;

  lua_getglobal(L, "class_pointer");  // push class pointer to the stack
  // grab the class pointer and cast it, so we can use it
  YamlPreprocessor * class_pointer =
    reinterpret_cast<YamlPreprocessor *>(lua_touserdata(L, lua_gettop(L)));
  lua_pop(L, 1);  // pop that class pointer from the stack

  if (lua_gettop(L) == 2 && !class_pointer->ros_node_->has_parameter(name)) {  // use default
    if (lua_isnumber(L, 2)) {
      lua_pushnumber(L, lua_tonumber(L, 2));
    } else if (lua_isboolean(L, 2)) {
      lua_pushboolean(L, lua_toboolean(L, 2));
    } else if (lua_isstring(L, 2)) {
      lua_pushstring(L, lua_tostring(L, 2));
    } else {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("Yaml Preprocessor"),
        "Couldn't load int/double/string value at param " << name);
      lua_pushnil(L);
    }
  } else {                                                 // no default
    if (!class_pointer->ros_node_->has_parameter(name)) {  // Push back a nil
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("Yaml Preprocessor"), "No rosparam found for: " << name);
      lua_pushnil(L);
    } else {
      if (!class_pointer->ros_node_->get_parameter(name, param)) {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("Yaml Preprocessor"), "Couldn't find a param with name " << name);
        lua_pushnil(L);
      }
      if (param.get_type() == rclcpp::PARAMETER_DOUBLE) {
        lua_pushnumber(L, param.as_double());
      } else if (param.get_type() == rclcpp::PARAMETER_INTEGER) {
        lua_pushinteger(L, param.as_int());
      } else if (param.get_type() == rclcpp::PARAMETER_STRING) {
        lua_pushstring(L, param.as_string().c_str());
      } else if (param.get_type() == rclcpp::PARAMETER_BOOL) {
        lua_pushstring(L, param.as_bool() ? "true" : "false");
      } else {
        RCLCPP_WARN_STREAM(
          rclcpp::get_logger("Yaml Preprocessor"),
          "Couldn't load int/double/string value at param " << name);
        lua_pushnil(L);
      }
    }
  }

  return 1;  // 1 return value
}
}  // namespace flatland_server