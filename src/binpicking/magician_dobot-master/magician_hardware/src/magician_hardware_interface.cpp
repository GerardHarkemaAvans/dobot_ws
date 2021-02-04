/*
Created on Thurs June 19 10:08 2019

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


#include <magician_hardware/magician_hardware_interface.h>

namespace magician_hardware {

MagicianHWInterface::MagicianHWInterface(): local_nh_("~")
{

    std::string default_magician_joints_array[3]={"magician_joint1", "magician_joint2", "magician_joint3"};
    std::vector<std::string> default_magician_joints(default_magician_joints_array, default_magician_joints_array+3);
    std::vector<std::string> magician_joints=local_nh_.param<std::vector<std::string> >("magician_joints", default_magician_joints);

    int default_pulse_signs_array[3]={-1, 1, -1};
    std::vector<int> default_pulse_signs(default_pulse_signs_array, default_pulse_signs_array+3);
    std::vector<int> pulse_signs=local_nh_.param<std::vector<int> >("pulse_signs", default_pulse_signs);

    magician_device_.reset(new MagicianDevice(magician_joints.size(), pulse_signs));

    simple_motors_.resize(magician_joints.size());
    for(size_t i=0; i<magician_joints.size(); i++)
    {
        simple_motors_[i].name=magician_joints[i];
    }

    for(size_t i=0; i<simple_motors_.size(); i++)
    {
        hardware_interface::JointStateHandle jnt_state_handle_tmp(simple_motors_[i].name,
                                                                  &simple_motors_[i].position,
                                                                  &simple_motors_[i].velocity,
                                                                  &simple_motors_[i].effort);
         jnt_state_interface_.registerHandle(jnt_state_handle_tmp);
    }
    registerInterface(&jnt_state_interface_);

    for(size_t i=0; i<simple_motors_.size(); i++)
    {
        hardware_interface::JointHandle jnt_hanlde_tmp(jnt_state_interface_.getHandle(simple_motors_[i].name),
                                                       &simple_motors_[i].position_cmd);
        jnt_position_cmd_interface_.registerHandle(jnt_hanlde_tmp);
    }
    registerInterface(&jnt_position_cmd_interface_);

    move_threshold_=M_PI/180;

}

MagicianHWInterface::~MagicianHWInterface()
{

}

bool MagicianHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
    root_nh_=root_nh;
    robot_hw_nh_=robot_hw_nh;

    if(!magician_device_->InitPose())
    {
        return false;
    }

    std::vector<double> jnt_values;
    if(!magician_device_->ReadPose(jnt_values))
    {
        return false;
    }

    for(size_t i=0; i<simple_motors_.size(); i++)
    {
        simple_motors_[i].position=jnt_values[i];
        simple_motors_[i].velocity=0;
        simple_motors_[i].effort=0;
        simple_motors_[i].position_cmd=jnt_values[i];
    }

    struct timespec read_update_tick;
    clock_gettime(CLOCK_REALTIME, &read_update_tick);
    read_update_time_.sec=read_update_tick.tv_sec;
    read_update_time_.nsec=read_update_tick.tv_nsec;

    return true;
}

void MagicianHWInterface::read(const ros::Time &time, const ros::Duration &period)
{
    read_update_dur_=time - read_update_time_;
    read_update_time_=time;

    std::vector<double> jnt_values;
    magician_device_->ReadPose(jnt_values);

    std::vector<double> pls_angles;
    magician_device_->GetPulseAngle(pls_angles);

    for(size_t i=0; i<simple_motors_.size(); i++)
    {
        simple_motors_[i].position=jnt_values[i];
        simple_motors_[i].velocity=pls_angles[i]/read_update_dur_.toSec();
    }
}

void MagicianHWInterface::write(const ros::Time &time, const ros::Duration &period)
{
    //std::cout << simple_motors_[0].position << std::endl;
    std::vector<double> jnt_cmds;
    jnt_cmds.resize(simple_motors_.size());

    for(size_t i=0; i<simple_motors_.size(); i++)
    {
        jnt_cmds[i]=simple_motors_[i].position_cmd;
    }

    magician_device_->WritePose(jnt_cmds);
}

bool MagicianHWInterface::reinitPose(const std::vector<double> &joint_values)
{
    if(joint_values.size()!=simple_motors_.size())
    {
        return false;
    }

    for(size_t i=0; i<simple_motors_.size(); i++)
    {
        simple_motors_[i].position=joint_values[i];
        simple_motors_[i].velocity=0;
        simple_motors_[i].effort=0;
        simple_motors_[i].position_cmd=joint_values[i];
    }

    return true;
}

bool MagicianHWInterface::isMoving()
{
    for(size_t i=0; i<simple_motors_.size(); i++)
    {
        if(fabs(simple_motors_[i].velocity)>move_threshold_)
        {
            return true;
        }
    }
    return false;
}

}
