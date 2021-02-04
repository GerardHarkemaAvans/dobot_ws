/*
Created on Thurs June 14 10:20 2019

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

#include <magician_background/magician_teleop_background.h>

namespace magician_background {

TeleopBackground::TeleopBackground(moveit::planning_interface::MoveGroupInterface *group, std::string action_name,
                                   planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor):
                                   group_(group), planning_scene_monitor_(planning_scene_monitor),
                                   local_nh_("~"), action_client_(action_name, true)
{
    goal_.trajectory.joint_names=group_->getActiveJoints();
    goal_.trajectory.header.stamp.sec=0;
    goal_.trajectory.header.stamp.nsec=0;

    joint_teleop_server_=local_nh_.advertiseService("joint_teleop", &TeleopBackground::jointTeleop_cb, this);
    cart_teleop_server_=local_nh_.advertiseService("cart_teleop", &TeleopBackground::cartTeleop_cb, this);
//    home_teleop_server_=local_nh_.advertiseService("home_teleop", &TeleopBackground::homeTeleop_cb, this);
    teleop_stop_server_=local_nh_.advertiseService("stop_teleop", &TeleopBackground::teleopStop_cb, this);

    // parameter for teleop
    joint_speed_limit_=1.57;
    joint_speed_default_=0.78;
    cart_duration_default_=0.04;
    resolution_angle_=0.02;
    resolution_linear_=0.005;

    cart_duration_=cart_duration_default_/0.4;
    joint_speed_=joint_speed_default_*0.4;
    joint_duration_=0.1;

    // active joints
    size_t active_joints[4]={0, 1, 3, 5};
    active_joints_=std::vector<size_t>(active_joints, active_joints+4);

    default_tip_link_=group_->getEndEffectorLink();
    root_link_=group_->getPlanningFrame();
}

TeleopBackground::~TeleopBackground()
{

}

bool TeleopBackground::jointTeleop_cb(magician_msgs::SetInt16::Request &req, magician_msgs::SetInt16::Response &resp)
{
    if(req.data==0 || abs(req.data)>goal_.trajectory.joint_names.size())
    {
        resp.success=false;
        resp.message="wrong joint teleop data";
        return true;
    }

    size_t active_joint_num=abs(req.data)-1;
    size_t joint_num=active_joints_[active_joint_num];
    std::vector<double> position_current=group_->getCurrentJointValues();
    std::vector<double> position_goal=position_current;
    double joint_current_position=position_current[joint_num];
    std::string direction=goal_.trajectory.joint_names[active_joint_num];

    double sign;
    if(abs(req.data)==req.data)
    {
        position_goal[joint_num]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[active_joint_num])->limits->upper;
        direction.append("+");
        sign=1;
    }
    else
    {
        position_goal[joint_num]=group_->getRobotModel()->getURDF()->getJoint(goal_.trajectory.joint_names[active_joint_num])->limits->lower;
        direction.append("-");
        sign=-1;
    }

    double duration_from_speed=fabs(position_goal[joint_num]-joint_current_position)/joint_speed_;
    if(duration_from_speed<=joint_duration_)
    {
        resp.success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp.message=result;
        return true;
    }

    trajectory_msgs::JointTrajectoryPoint point_tmp;

    robot_state::RobotStatePtr kinematic_state_ptr=group_->getCurrentState();
    robot_state::RobotState kinematic_state=*kinematic_state_ptr;
    const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

    planning_scene_monitor_->updateFrameTransforms();
    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();

    std::vector<double> position_tmp=position_current;
    bool collision_flag=false;

    std::vector<double> static_velocities;
    static_velocities.resize(goal_.trajectory.joint_names.size());
    for(size_t i=0; i<static_velocities.size(); i++)
    {
        static_velocities[i]=0;
    }

    std::vector<double> dynamic_velocities;
    dynamic_velocities=static_velocities;
    dynamic_velocities[active_joint_num]=joint_speed_*sign;

    std::vector<double> accelerations;
    accelerations=static_velocities;

    int loop_num=1;
    while(fabs(position_goal[joint_num]-position_tmp[joint_num])/joint_speed_>joint_duration_)
    {
        if(loop_num==1)
        {
            position_tmp[joint_num]+=0.5*joint_speed_*joint_duration_*sign;
        }
        else
        {
            position_tmp[joint_num]+=joint_speed_*joint_duration_*sign;
        }

        kinematic_state.setJointGroupPositions(joint_model_group, position_tmp);
        if(plan_scene->isStateColliding(kinematic_state, group_->getName()))
        {
            if(loop_num==1)
            {
                resp.success=false;
                std::string result="robot can't move in ";
                result.append(direction);
                result.append(" direction any more");
                resp.message=result;
                return true;
            }
            collision_flag=true;
            break;
        }

        point_tmp.time_from_start=ros::Duration(joint_duration_*loop_num);
        std::vector<double> pos_tmp;
        pos_tmp.resize(goal_.trajectory.joint_names.size());
        for(size_t i=0; i<pos_tmp.size(); i++)
        {
            pos_tmp[i]=position_tmp[active_joints_[i]];
        }
        point_tmp.positions=pos_tmp;
        point_tmp.velocities=dynamic_velocities;
        point_tmp.accelerations=accelerations;
        goal_.trajectory.points.push_back(point_tmp);
        loop_num++;
    }

    if(!collision_flag)
    {
        kinematic_state.setJointGroupPositions(joint_model_group, position_goal);
        if(!plan_scene->isStateColliding(kinematic_state, group_->getName()))
        {
            size_t traj_length=goal_.trajectory.points.size();
            std::vector<double> pos_tmp;
            pos_tmp.resize(goal_.trajectory.joint_names.size());
            for(size_t i=0; i<pos_tmp.size(); i++)
            {
                pos_tmp[i]=position_goal[active_joints_[i]];
            }
            point_tmp.positions=pos_tmp;
            point_tmp.velocities=static_velocities;
            point_tmp.accelerations=accelerations;
            double last_time=goal_.trajectory.points[traj_length-1].time_from_start.toSec();
            double extra_time=2*(pos_tmp[active_joint_num]-goal_.trajectory.points[traj_length-1].positions[active_joint_num])/(joint_speed_*sign);
            if(extra_time>joint_duration_)
            {
                ros::Duration dur(last_time+extra_time);
                point_tmp.time_from_start=dur;
                goal_.trajectory.points.push_back(point_tmp);
            }
            else
            {
                ros::Duration dur(last_time+joint_duration_+extra_time);
                point_tmp.time_from_start=dur;
                goal_.trajectory.points[traj_length-1]=point_tmp;
            }
        }
        else
        {
            size_t traj_length=goal_.trajectory.points.size();
            double last_time=goal_.trajectory.points[traj_length-1].time_from_start.toSec();
            ros::Duration dur(last_time+joint_duration_);
            goal_.trajectory.points[traj_length-1].velocities=static_velocities;
            goal_.trajectory.points[traj_length-1].time_from_start=dur;
        }
    }
    else
    {
        size_t traj_length=goal_.trajectory.points.size();
        double last_time=goal_.trajectory.points[traj_length-1].time_from_start.toSec();
        ros::Duration dur(last_time+joint_duration_);
        goal_.trajectory.points[traj_length-1].velocities=static_velocities;
        goal_.trajectory.points[traj_length-1].time_from_start=dur;
    }

    action_client_.sendGoal(goal_);
    goal_.trajectory.points.clear();

    resp.success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp.message=result;
    return true;
}

bool TeleopBackground::cartTeleop_cb(magician_msgs::SetInt16::Request &req, magician_msgs::SetInt16::Response &resp)
{
    if(req.data==0 || abs(req.data)>goal_.trajectory.joint_names.size())
    {
        resp.success=false;
        resp.message="wrong cart. teleop data";
        return true;
    }
    int operation_num=abs(req.data);
    int symbol;
    if(operation_num==req.data)
        symbol=1;
    else
        symbol=-1;

    geometry_msgs::PoseStamped current_pose=group_->getCurrentPose(default_tip_link_);
    std::vector<double> current_joint_states=group_->getCurrentJointValues();

    tf::Pose tf_pose_tmp;
    Eigen::Affine3d affine_pose_tmp;
    Eigen::Affine3d affine_current_pose;

    tf::Vector3 x_axis(1, 0, 0);
    tf::Vector3 y_axis(0, 1, 0);
    tf::Vector3 z_axis(0, 0, 1);

    robot_state::RobotStatePtr kinematic_state_ptr=group_->getCurrentState();
    robot_state::RobotState kinematic_state=*kinematic_state_ptr;
    const robot_state::JointModelGroup* joint_model_group = kinematic_state.getJointModelGroup(group_->getName());

    planning_scene_monitor_->updateFrameTransforms();
    planning_scene::PlanningSceneConstPtr plan_scene=planning_scene_monitor_->getPlanningScene();

    trajectory_msgs::JointTrajectoryPoint point_tmp;
    point_tmp.positions.resize(goal_.trajectory.joint_names.size());

    std::string direction;

    bool ik_have_result=true;
    double resolution_alpha=resolution_angle_;
    double resolution_delta=resolution_linear_;
    double velocity_factor;

    if(cart_duration_>0)
    {
        velocity_factor=1/cart_duration_;
    }
    else
    {
        resp.success=false;
        std::string result="robot can't move";
        resp.message=result;
        return true;
    }

    std::vector<double> static_velocities;
    static_velocities.resize(goal_.trajectory.joint_names.size());
    for(size_t i=0; i<static_velocities.size(); i++)
    {
        static_velocities[i]=0;
    }

    std::vector<double> last_velocities;
    last_velocities=static_velocities;

    std::vector<double> displacements;
    displacements=static_velocities;

    std::vector<double> accelerations;
    accelerations=static_velocities;

    size_t loop_num=1;
    while(ik_have_result)
    {
        switch (operation_num) {
        case 1:
            current_pose.pose.position.x+=symbol*resolution_delta;
            if(symbol==1)
                direction="X+";
            else
                direction="X-";
            break;
        case 2:
            current_pose.pose.position.y+=symbol*resolution_delta;
            if(symbol==1)
                direction="Y+";
            else
                direction="Y-";
            break;
        case 3:
            current_pose.pose.position.z+=symbol*resolution_delta;
            if(symbol==1)
                direction="Z+";
            else
                direction="Z-";
            break;
        case 4:
            PoseStampedRotation(current_pose, z_axis, symbol*resolution_alpha);
            if(symbol==1)
                direction="Rz+";
            else
                direction="Rz-";
            break;
        default:
            break;
        }
        tf::poseMsgToTF(current_pose.pose, tf_pose_tmp);
        tf::poseTFToEigen(tf_pose_tmp, affine_pose_tmp);
        affine_current_pose=affine_pose_tmp;

        Eigen::Isometry3d iso_current_pose;
        iso_current_pose.translation()=affine_current_pose.translation();
        iso_current_pose.linear()=affine_current_pose.rotation();

        ik_have_result=kinematic_state.setFromIK(joint_model_group, iso_current_pose, default_tip_link_);
        affine_current_pose=iso_current_pose.matrix();

        if(ik_have_result)
        {
            if(goal_.trajectory.points.size()!=loop_num-1)
            {
                break;
            }
            double biggest_shift=0;
            for(size_t j=0; j<goal_.trajectory.joint_names.size(); j++)
            {
                point_tmp.positions[j]=*kinematic_state.getJointPositions(goal_.trajectory.joint_names[j]);
                if(loop_num==1)
                {
                    last_velocities=static_velocities;
                    displacements[j]=point_tmp.positions[j]-current_joint_states[active_joints_[j]];
                    double shift_tmp=fabs(displacements[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
                else {
                    last_velocities=goal_.trajectory.points[loop_num-2].velocities;
                    displacements[j]=point_tmp.positions[j]-goal_.trajectory.points[loop_num-2].positions[j];
                    double shift_tmp=fabs(displacements[j]);
                    if(shift_tmp>biggest_shift)
                        biggest_shift=shift_tmp;
                }
            }
            if(biggest_shift>cart_duration_*joint_speed_limit_)
            {
                break;
            }
            if(plan_scene->isStateColliding(kinematic_state, group_->getName()))
            {
                break;
            }
            point_tmp.velocities.resize(displacements.size());
            for(size_t i=0; i<displacements.size(); i++)
            {
                if(loop_num==1)
                {
                    point_tmp.velocities[i]=displacements[i]*velocity_factor;
                }
                else
                {
                    point_tmp.velocities[i]=2*displacements[i]*velocity_factor-last_velocities[i];
                }
            }
            point_tmp.accelerations=accelerations;
            ros::Duration dur((loop_num+1)*cart_duration_);
            point_tmp.time_from_start=dur;
            goal_.trajectory.points.push_back(point_tmp);
        }
        else
        {
            break;
        }

        loop_num++;
    }

    if(goal_.trajectory.points.size()==0)
    {
        resp.success=false;
        std::string result="robot can't move in ";
        result.append(direction);
        result.append(" direction any more");
        resp.message=result;
        return true;
    }
    else
    {
        size_t traj_length=goal_.trajectory.points.size();
        double last_time=goal_.trajectory.points[traj_length-1].time_from_start.toSec();
        ros::Duration dur(last_time+cart_duration_);
        goal_.trajectory.points[traj_length-1].velocities=static_velocities;
        goal_.trajectory.points[traj_length-1].time_from_start=dur;
    }

    action_client_.sendGoal(goal_);
    goal_.trajectory.points.clear();

    resp.success=true;
    std::string result="robot is moving in ";
    result.append(direction);
    result.append(" direction");
    resp.message=result;
    return true;
}

bool TeleopBackground::teleopStop_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &resp)
{
    goal_.trajectory.points.clear();
    action_client_.sendGoal(goal_);

    resp.success=true;
    resp.message="stop moving";
    return true;
}

void TeleopBackground::PoseStampedRotation(geometry_msgs::PoseStamped &pose_stamped, const tf::Vector3 &axis, double angle)
{
    tf::Quaternion q_1(pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y,
                                 pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w);
    tf::Quaternion q_2(axis, angle);
    tf::Matrix3x3 m(q_1);
    tf::Matrix3x3 m_2(q_2);
    m_2.operator *=(m);
    double r, p, y;
    m_2.getRPY(r,p,y);

    q_2.setRPY(r, p, y);
    pose_stamped.pose.orientation.x=q_2.getX();
    pose_stamped.pose.orientation.y=q_2.getY();
    pose_stamped.pose.orientation.z=q_2.getZ();
    pose_stamped.pose.orientation.w=q_2.getW();
}

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"magician_background", ros::init_options::AnonymousName);

    ros::CallbackQueue move_group_cb_queue;

    std::string move_group_name="magician_arm";
    std::string move_group_desc="robot_description";
    ros::NodeHandle move_group_nh;
    move_group_nh.setCallbackQueue(&move_group_cb_queue);

    moveit::planning_interface::MoveGroupInterface::Options move_group_options(move_group_name, move_group_desc, move_group_nh);

    moveit::planning_interface::MoveGroupInterface move_group(move_group_options);

    ros::AsyncSpinner move_group_spinner(1, &move_group_cb_queue);
    move_group_spinner.start();

    move_group.startStateMonitor();

    move_group.getCurrentJointValues();

    ros::AsyncSpinner common_spinner(1);
    common_spinner.start();

    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();

    magician_background::TeleopBackground teleop_background(&move_group, "magician_arm_controller/follow_joint_trajectory", planning_scene_monitor);

    ros::waitForShutdown();

    return 0;

}
