/*
Created on Thurs June 14 10:15 2019

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

#ifndef MAGICIAN_TELEOP_BACKGROUND_H
#define MAGICIAN_TELEOP_BACKGROUND_H

#include <ros/ros.h>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <magician_msgs/SetInt16.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace magician_background {

class TeleopBackground
{
public:
    TeleopBackground(moveit::planning_interface::MoveGroupInterface *group, std::string action_name,
                     planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);
    ~TeleopBackground();

    bool jointTeleop_cb(magician_msgs::SetInt16::Request &req, magician_msgs::SetInt16::Response &resp);
    bool cartTeleop_cb(magician_msgs::SetInt16::Request &req, magician_msgs::SetInt16::Response &resp);
    bool homeTeleop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);
    bool teleopStop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp);

    void PoseStampedRotation(geometry_msgs::PoseStamped &pose_stamped, const tf::Vector3 &axis, double angle);

private:
    moveit::planning_interface::MoveGroupInterface *group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    ros::NodeHandle root_nh_, local_nh_;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client_;
    control_msgs::FollowJointTrajectoryGoal goal_;

    ros::ServiceServer joint_teleop_server_;
    ros::ServiceServer cart_teleop_server_;
    ros::ServiceServer home_teleop_server_;
    ros::ServiceServer teleop_stop_server_;

    std::vector<size_t> active_joints_;

    double joint_speed_limit_;
    double velocity_scaling_;
    double joint_speed_default_;
    double cart_duration_default_;

    double resolution_angle_;
    double resolution_linear_;
    double cart_duration_;
    double joint_speed_;
    double joint_duration_;

    std::string default_tip_link_;
    std::string root_link_;
};

}

#endif
