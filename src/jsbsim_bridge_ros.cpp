
/****************************************************************************
 *
 *   Copyright (c) 2020 Auterion AG. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 */

#include "jsbsim_bridge_ros.h"

using namespace Eigen;
using namespace std;

// Constructor
JSBSimBridgeRos::JSBSimBridgeRos(const rclcpp::NodeOptions& options)
  : rclcpp::Node("jsbsim_bridge_ros", options) {
  
  this->declare_parameter<std::string>("config", std::string(JSBSIM_ROOT_DIR) + "/configs/quadrotor_x.xml");
  this->declare_parameter<std::string>("script", std::string(JSBSIM_ROOT_DIR) + "/scene/LSZH.xml");
  this->declare_parameter<double>("dt", 0.004);

  this->get_parameter("config", path);
  this->get_parameter("script", script_path);
  double dt;
  this->get_parameter("dt", dt);

  simloop_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt),
    std::bind(&JSBSimBridgeRos::simloopCallback, this));
  statusloop_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&JSBSimBridgeRos::statusloopCallback, this));

  config_.ParseEnvironmentVariables();
  config_.ParseConfigFile(path);
  config_.setInitScriptPath(script_path);
  config_.setHeadless(false);  // Set false for FlightGear

  fdmexec_ = new JSBSim::FGFDMExec();
  
  // Enable FlightGear output XML
  fdmexec_->SetOutputDirectives(SGPath("/home/nathan/jsbsim_ros2/src/px4-jsbsim-bridge/data_out/flightgear.xml"));

  jsbsim_bridge_ = std::make_unique<JSBSimBridge>(fdmexec_, config_);
}


JSBSimBridgeRos::~JSBSimBridgeRos() {
  // Destructor
  delete fdmexec_;
}

void JSBSimBridgeRos::simloopCallback() {
  if (jsbsim_bridge_) {
  jsbsim_bridge_->Run();
  }
}

void JSBSimBridgeRos::statusloopCallback() {
  // TODO: Publish simulation status
}
