/*
Created on Thurs June 19 16:31 2019

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

#ifndef MAGICIAN_DEVICE_H
#define MAGICIAN_DEVICE_H

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <DobotDll.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace magician_hardware {

const double RAD_PER_PULSE=M_PI_2/8000;
const double PULSE_PER_RAD=8000/M_PI_2;
const double RAD_PER_DEGREE=M_PI/180;

class MagicianDevice {
public:
    MagicianDevice(unsigned long motor_num, std::vector<int> pulse_signs);
    ~MagicianDevice();

    bool InitPose();
    bool ResetPose(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp, std::vector<double> &joint_values);
    bool ReadPose(std::vector<double> &joint_values);
    bool WritePose(const std::vector<double> &joint_cmds);
    void GetPulseAngle(std::vector<double> &pulse_angles);

private:
    ros::NodeHandle local_nh_;

    unsigned long motor_num_;
    std::vector<int> pulse_signs_;

    std::vector<double> joint_bases_;
    std::vector<double> joint_offsets_;

    std::vector<double> pulse_angles_;
    std::vector<double> angle_degrees_last;
};

}

#endif
