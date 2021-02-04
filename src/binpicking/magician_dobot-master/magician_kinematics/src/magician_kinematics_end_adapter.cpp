/*
Created on Thurs June 13 10:58 2019

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

#include <magician_kinematics/magician_kinematics_end_adapter.h>

namespace magician_kinematics {

EndAdapter::EndAdapter(const double &end_angle, const KDL::Vector &eef_trans, const double &eef_angle,
                       const bool &eef_positive, const bool &eef_active)
{
    end_angle_=end_angle;
    eef_trans_=eef_trans;
    eef_angle_=eef_angle;
    if(eef_positive){
        eef_sign_=1;
    }
    else {
        eef_sign_=-1;
    }
    eef_active_=eef_active;

    vector_z_=KDL::Vector(0, 0, 1);
}

EndAdapter::~EndAdapter()
{

}

bool EndAdapter::AdaptFK(KDL::Vector &trans, KDL::Rotation &rot, const double &angle1, const double &angle4)
{
    if(eef_active_)
    {
        rot=KDL::Rotation::Rot(vector_z_, end_angle_+eef_angle_+angle1+eef_sign_*angle4);
    }
    else
    {
        rot=KDL::Rotation::Rot(vector_z_, end_angle_+eef_angle_);
    }

    trans+=KDL::Rotation::Rot(vector_z_, end_angle_+angle1)*eef_trans_;

    return true;
}

bool EndAdapter::AdaptIK(KDL::Vector &trans, KDL::Rotation &rot, double &angle4)
{
    double module_xy=sqrt(trans.x()*trans.x()+trans.y()*trans.y());
    if(eef_trans_.y()>module_xy || module_xy<LIMIT_TOLERANCE)
    {
        return false;
    }

    double trans_offset_angle=asin(eef_trans_.y()/module_xy);
    double angle1=atan2(trans.y(), trans.x());
    angle1-=trans_offset_angle;
    angle1-=end_angle_;

    trans-=KDL::Rotation::Rot(vector_z_, end_angle_+angle1)*eef_trans_;

    KDL::Vector rot_vector;
    double rot_angle;
    rot_angle=rot.GetRotAngle(rot_vector);

    rot_vector.Normalize();
    if(fabs(KDL::dot(vector_z_, rot_vector))<cos(0.017))
    {
        return false;
    }

    if(rot_vector.z()<0)
    {
        rot_angle*=-1;
    }

    rot=KDL::Rotation::Rot(vector_z_, rot_angle-eef_angle_);

    if(eef_active_)
    {
        angle4=eef_sign_*(rot_angle-end_angle_-angle1-eef_angle_);
    }
    else
    {
        angle4=-1*eef_sign_*angle1;
    }

    return true;
}

}
