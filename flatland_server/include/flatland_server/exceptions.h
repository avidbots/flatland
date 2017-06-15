/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.h
 * @brief	 Defines flatland Layer
 * @author Chunshang Li
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

#ifndef FLATLAND_SERVER_EXCEPTIONS_H
#define FLATLAND_SERVER_EXCEPTIONS_H

#include <string>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

namespace flatland_server {

class Exception : public std::runtime_error {
  public:
    Exception(const std::string &msg) : std::runtime_error(msg) {}
};

class YAMLException : public Exception {
  public:
    YAML::Mark yaml_cpp_mark_;
    std::string msg_, yaml_cpp_msg_;

    YAMLException(const std::string &msg, const std::string &yaml_cpp_msg, 
      const YAML::Mark &yaml_cpp_mark) :
      Exception(error_msg(msg, yaml_cpp_msg, yaml_cpp_mark)),
      yaml_cpp_msg_(yaml_cpp_msg), yaml_cpp_mark_(yaml_cpp_mark){}

    YAMLException(const std::string &msg) :
      Exception(msg){}
      
  private:
    static const std::string error_msg(const std::string &msg,
      const std::string &yaml_cpp_msg, 
      const YAML::Mark &yaml_cpp_mark) {
        std::stringstream output;

        output << msg;
        if (yaml_cpp_mark.pos == -1 && 
            yaml_cpp_mark.line == -1 && 
            yaml_cpp_mark.column == -1) {
          output << " Error at Line " << yaml_cpp_mark.line + 1 << ", col "
                 << yaml_cpp_mark.column + 1;
        }

        if (yaml_cpp_msg.size() > 0) {
          output << "" << yaml_cpp_msg;
        }
      }
};

}; // namespace flatland_server

#endif  // FLATLAND_SERVER_WORLD_H
