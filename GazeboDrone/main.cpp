/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include <chrono>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

constexpr int NWIDTH = 7;

using namespace msr::airlib;

msr::airlib::MultirotorRpcLibClient client;


void cbLocalPose(ConstPosesStampedPtr &_msg) {
  std::cout << std::fixed;
  std::cout << std::setprecision(3);

  for ( int i = 0; i < _msg->pose_size(); i++ ) {
	  std::cout << "local (" << std::setw(2) << i << ") ";
    std::cout << std::left << std::setw(32) << _msg->pose(i).name();
    auto x = _msg->pose(i).position().x();
    auto y = _msg->pose(i).position().y();
    auto z = _msg->pose(i).position().z();
    auto ow = _msg->pose(i).orientation().w();
    auto ox = _msg->pose(i).orientation().x();
    auto oy = _msg->pose(i).orientation().y();
    auto oz = _msg->pose(i).orientation().z();
    std::cout << " x: " << std::right << std::setw(NWIDTH) << x;
    std::cout << " y: " << std::right << std::setw(NWIDTH) << y;
    std::cout << " z: " << std::right << std::setw(NWIDTH) << z;

    std::cout << " ow: " << std::right << std::setw(NWIDTH) << ow;
    std::cout << " ox: " << std::right << std::setw(NWIDTH) << ox;
    std::cout << " oy: " << std::right << std::setw(NWIDTH) << oy;
    std::cout << " oz: " << std::right << std::setw(NWIDTH) << oz;
    std::cout << std::endl;
    if( i == _msg->pose_size()-1 ){ 
        msr::airlib::Vector3r p;
        msr::airlib::Quaternionr o;
	p.x() = x;
        p.y() = y;
	p.z() = z;
	o.w() = ow;
	o.x() = ox;
	o.y() = oy;
	o.z() = oz;
    	client.simSetVehiclePose(Pose(p,o), true);
    }
   
  }
  
  std::cout << std::endl;
}

void cbGobalPose(ConstPosesStampedPtr &_msg) {
  std::cout << std::fixed;
  std::cout << std::setprecision(4);

  for ( int i = 0; i < _msg->pose_size(); i++ ) {
    std::cout << "global (" << i << ") ";
    std::cout << std::left << std::setw(32) << _msg->pose(i).name();
    std::cout << " x: " << std::right << std::setfill(' ') << std::setw(NWIDTH) << _msg->pose(i).position().x();
    std::cout << " y: " << std::right << std::setfill(' ') << std::setw(NWIDTH) << _msg->pose(i).position().y();
    std::cout << " z: " << std::right << std::setfill(' ') << std::setw(NWIDTH) << _msg->pose(i).position().z();

    std::cout << " ow: " << std::right << std::setfill(' ') << std::setw(NWIDTH) << _msg->pose(i).orientation().w();
    std::cout << " ox: " << std::right << std::setfill(' ') << std::setw(NWIDTH) << _msg->pose(i).orientation().x();
    std::cout << " oy: " << std::right << std::setfill(' ') << std::setw(NWIDTH) << _msg->pose(i).orientation().y();
    std::cout << " oz: " << std::right << std::setfill(' ') << std::setw(NWIDTH) << _msg->pose(i).orientation().z();
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

int main(int _argc, char **_argv) {

  
  client.confirmConnection();

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo topics
  gazebo::transport::SubscriberPtr subPose1 = node->Subscribe("~/pose/local/info", cbLocalPose);
  gazebo::transport::SubscriberPtr subPose2 = node->Subscribe("~/pose/info", cbGobalPose);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
