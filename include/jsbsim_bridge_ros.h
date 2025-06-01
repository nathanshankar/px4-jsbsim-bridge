/****************************************************************************
 *
 *   Copyright (c) 2020 Auterion AG. All rights reserved.
 *   (License text unchanged)
 ****************************************************************************/

/**
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 */

#ifndef JSBSIM_BRIDGE_ROS_H
#define JSBSIM_BRIDGE_ROS_H

#include "jsbsim_bridge.h"

#include <rclcpp/rclcpp.hpp>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class JSBSimBridgeRos : public rclcpp::Node {
 public:
  JSBSimBridgeRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~JSBSimBridgeRos();

 private:
  void simloopCallback();
  void statusloopCallback();

  rclcpp::TimerBase::SharedPtr simloop_timer_, statusloop_timer_;

  JSBSim::FGFDMExec* fdmexec_;
  ConfigurationParser config_;
  std::unique_ptr<JSBSimBridge> jsbsim_bridge_;

  std::string path;
  std::string script_path;
};

#endif
