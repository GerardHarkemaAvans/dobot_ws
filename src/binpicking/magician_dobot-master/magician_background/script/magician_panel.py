#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 18 10:32:22 2019

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
"""

from __future__ import division
import rospy
import math
import tf
import moveit_commander
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from magician_msgs.srv import SetInt16, SetInt16Request
import wx
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
import dynamic_reconfigure.client

class MyFrame(wx.Frame):
    def __init__(self,parent,id):  
        the_size=(700, 270)
        wx.Frame.__init__(self,parent,id,'Magician Control Panel',pos=(250,100)) 
        self.panel=wx.Panel(self)
        font=self.panel.GetFont()
        font.SetPixelSize((10, 20))
        self.panel.SetFont(font)
        
        self.listener = tf.TransformListener()
        
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.group=moveit_commander.MoveGroupCommander('magician_arm')
        
        self.elfin_basic_api_ns='magician_background/'
    
        self.joint_names=self.group.get_active_joints()
        self.active_joints=[0, 1, 3, 5]
        
        self.ref_link_name=self.group.get_planning_frame()
        self.end_link_name=self.group.get_end_effector_link()
        
        self.js_display=[0]*4 # joint_states
        self.jm_button=[0]*4 # joints_minus
        self.jp_button=[0]*4 # joints_plus
        self.js_label=[0]*4 # joint_states
                      
        self.ps_display=[0]*4 # pcs_states
        self.pm_button=[0]*4 # pcs_minus
        self.pp_button=[0]*4 # pcs_plus
        self.ps_label=[0]*4 # pcs_states
                      
        self.display_init()
        
        self.call_teleop_joint=rospy.ServiceProxy(self.elfin_basic_api_ns+'joint_teleop', 
                                                  SetInt16)
        self.call_teleop_joint_req=SetInt16Request()
        
        self.call_teleop_cart=rospy.ServiceProxy(self.elfin_basic_api_ns+'cart_teleop', 
                                                 SetInt16)
        self.call_teleop_cart_req=SetInt16Request()
        
        self.call_teleop_stop=rospy.ServiceProxy(self.elfin_basic_api_ns+'stop_teleop', 
                                                 SetBool)
        self.call_teleop_stop_req=SetBoolRequest()
        
        self.call_stop=rospy.ServiceProxy(self.elfin_basic_api_ns+'stop_teleop', 
                                          SetBool)
        self.call_stop_req=SetBoolRequest()
        self.call_stop_req.data=True
        
        self.SetMinSize(the_size)
        self.SetMaxSize(the_size)
        
        rospy.Timer(rospy.Duration(nsecs=50000000), self.monitor_status)
        
    def display_init(self):
        js_pos=[20, 20]
        js_btn_length=[70, 70, 61, 80]
        js_distances=[10, 20, 10, 26]
        dis_h=50
        for i in xrange(len(self.js_display)):
            self.jp_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' +', 
                                        pos=(js_pos[0],
                                             js_pos[1]+(3-i)*dis_h),
                                        size=(70,40))
            dis_tmp=js_btn_length[0]+js_distances[0]
                                        
            self.jp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_joints(evt, mark) )
            self.jp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
            
            self.jm_button[i]=wx.Button(self.panel,
                                        label='J'+str(i+1)+' -', 
                                        pos=(js_pos[0]+dis_tmp,
                                             js_pos[1]+(3-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=js_btn_length[1]+js_distances[1]
                                        
            self.jm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_joints(evt, mark) )
            self.jm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            pos_js_label=(js_pos[0]+dis_tmp, js_pos[1]+(3-i)*dis_h)
            self.js_label[i]=wx.StaticText(self.panel,
                                           label='J'+str(i+1)+'/deg:',
                                           pos=pos_js_label)
            self.js_label[i].SetPosition((pos_js_label[0], pos_js_label[1]+abs(40-self.js_label[i].GetSize()[1])/2))
            dis_tmp+=js_btn_length[2]+js_distances[2]

            pos_js_display=(js_pos[0]+dis_tmp, js_pos[1]+(3-i)*dis_h)
            self.js_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value='', 
                                           pos=pos_js_display)
            self.js_display[i].SetPosition((pos_js_display[0], pos_js_display[1]+abs(40-self.js_display[i].GetSize()[1])/2))
            dis_tmp+=js_btn_length[3]+js_distances[3]

        ps_pos=[js_pos[0]+dis_tmp, 20]
        ps_btn_length=[70, 70, 53, 80]
        ps_distances=[10, 20, 10, 20]
        pcs_btn_label=['X', 'Y', 'Z', 'Rz']
        pcs_label=['X', 'Y', 'Z', 'Rz']
        unit_label=['/mm:', '/mm:', '/mm:', '/deg:']
        for i in xrange(len(self.ps_display)):
            self.pp_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' +', 
                                        pos=(ps_pos[0],
                                             ps_pos[1]+(3-i)*dis_h),
                                        size=(70,40))
            dis_tmp=ps_btn_length[0]+ps_distances[0]
                                        
            self.pp_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=i+1 : self.teleop_pcs(evt, mark) )
            self.pp_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=i+1 : self.release_button(evt, mark) )
            
            self.pm_button[i]=wx.Button(self.panel,
                                        label=pcs_btn_label[i]+' -', 
                                        pos=(ps_pos[0]+dis_tmp,
                                             ps_pos[1]+(3-i)*dis_h),
                                        size=(70,40))
            dis_tmp+=ps_btn_length[1]+ps_distances[1]
                                        
            self.pm_button[i].Bind(wx.EVT_LEFT_DOWN, 
                                   lambda evt, mark=-1*(i+1) : self.teleop_pcs(evt, mark) )
            self.pm_button[i].Bind(wx.EVT_LEFT_UP,
                                   lambda evt, mark=-1*(i+1) : self.release_button(evt, mark) )
            
            pos_ps_label=(ps_pos[0]+dis_tmp, ps_pos[1]+(3-i)*dis_h)
            self.ps_label[i]=wx.StaticText(self.panel, 
                                           label=pcs_label[i]+unit_label[i],
                                           pos=pos_ps_label)
            self.ps_label[i].SetPosition((pos_ps_label[0], pos_ps_label[1]+abs(40-self.ps_label[i].GetSize()[1])/2))
            dis_tmp+=ps_btn_length[2]+ps_distances[2]
            
            pos_ps_display=(ps_pos[0]+dis_tmp, ps_pos[1]+(3-i)*dis_h)
            self.ps_display[i]=wx.TextCtrl(self.panel, 
                                           style=(wx.TE_CENTER |wx.TE_READONLY),
                                           value='', 
                                           pos=pos_ps_display)
            self.ps_display[i].SetPosition((pos_ps_display[0], pos_ps_display[1]+abs(40-self.ps_display[i].GetSize()[1])/2))
            dis_tmp+=ps_btn_length[3]+ps_distances[3]
            
    def teleop_joints(self,event,mark):       
        self.call_teleop_joint_req.data=mark
        resp=self.call_teleop_joint.call(self.call_teleop_joint_req)
#        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
        
    def teleop_pcs(self,event,mark): 
        self.call_teleop_cart_req.data=mark            
        resp=self.call_teleop_cart.call(self.call_teleop_cart_req)
#        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
    
    def release_button(self, event, mark):
        self.call_teleop_stop_req.data=True
        resp=self.call_teleop_stop.call(self.call_teleop_stop_req)
#        wx.CallAfter(self.update_reply_show, resp)
        event.Skip()
        
    def updateDisplay(self, msg):      
        for i in xrange(len(self.js_display)):
            self.js_display[i].SetValue(msg[i])
        
        for i in xrange(len(self.ps_display)):
            self.ps_display[i].SetValue(msg[i+4])
                
    def monitor_status(self, evt):
        self.key=[]
        
        current_joint_values=self.group.get_current_joint_values()
        for i in xrange(len(self.active_joints)):
            self.key.append(str(round(current_joint_values[self.active_joints[i]]*180/math.pi, 2)))
            
        ref_link=self.ref_link_name
        end_link=self.end_link_name
        
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform(ref_link, end_link, rospy.Time(0), rospy.Duration(100))
                (xyz,qua) = self.listener.lookupTransform(ref_link, end_link, rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            
        rpy=tf.transformations.euler_from_quaternion(qua)
            
        self.key.append(str(round(xyz[0]*1000, 2)))
        self.key.append(str(round(xyz[1]*1000, 2)))
        self.key.append(str(round(xyz[2]*1000, 2)))
        
        self.key.append(str(round(rpy[2]*180/math.pi, 2)))
        
        wx.CallAfter(self.updateDisplay, self.key)

if __name__=='__main__':  
    rospy.init_node('magician_panel')
    app=wx.App(False)  
    myframe=MyFrame(parent=None,id=-1)  
    myframe.Show(True)

    app.MainLoop()