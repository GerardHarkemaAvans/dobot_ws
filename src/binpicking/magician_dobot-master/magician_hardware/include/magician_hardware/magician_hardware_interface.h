/*
Created on Thurs June 19 18:18 2019

@author: Cong Liu

 Software License Agreement (BSD License)

 Copyright (c) 2019, Dobot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
// author: Cong Liu

#ifndef MAGICIAN_HARDWARE_INTERFACE_H
#define MAGICIAN_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <pthread.h>
#include <time.h>
#include <string>
#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

#include <magician_hardware/magician_device.h>

namespace magician_hardware {

typedef struct{
    std::string name;

    double position;
    double velocity;
    double effort;

    double position_cmd;
}SimpleMotor;

class MagicianHWInterface : public hardware_interface::RobotHW
{
public:
    MagicianHWInterface();
    ~MagicianHWInterface();

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

    bool reinitPose(const std::vector<double> &joint_values);
    bool isMoving();
    bool ResetPose(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

private:
    boost::shared_ptr<MagicianDevice> magician_device_;
    std::vector<SimpleMotor> simple_motors_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::PositionJointInterface jnt_position_cmd_interface_;

    ros::NodeHandle root_nh_, local_nh_, robot_hw_nh_;

    ros::Time read_update_time_;
    ros::Duration read_update_dur_;

    double move_threshold_;
};

}

#endif
