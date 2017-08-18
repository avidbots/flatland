/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	simulation_manager.h
 * @brief	Simulation manager class header definition
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

#ifndef FLATLAND_SERVER_SIMULATION_MANAGER_H
#define FLATLAND_SERVER_SIMULATION_MANAGER_H

#include <Box2D/Box2D.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world.h>
#include <string>

namespace flatland_server {

class SimulationManager {
 public:
  bool run_simulator_;           ///<  While true, keep running the sim loop
  World *world_;                 ///< Simulation world
  double update_rate_;           ///< sim loop rate
  double step_size_;             ///< step size
  bool show_viz_;                ///< flag to determine if to show visualization
  double viz_pub_rate_;          ///< rate to publish visualization
  std::string world_yaml_file_;  ///< path to the world file

  /**
   * @name  Simulation Manager constructor
   * @param[in] world_file The path to the world.yaml file we wish to load
   * @param[in] update_rate Simulator loop rate
   * @param[in] step_size Time to step each iteration
   * @param[in] show_viz if to show visualization
   * @param[in] viz_pub_rate rate to publish visualization
   * behaving ones
   */
  SimulationManager(std::string world_yaml_file, double update_rate,
                    double step_size, bool show_viz, double viz_pub_rate);

  /**
   * This method contains the loop that runs the simulation
   */
  void Main();

  /**
   * Kill the world
   */
  void Shutdown();
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_SIMULATION_MANAGER_H
