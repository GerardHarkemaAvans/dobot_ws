#!/usr/bin/env python

import rospy
import math
import random
from geometry_msgs.msg import PoseStamped

from flexbe_core import EventState, Logger
import getpass

user = getpass.getuser()


class test( object ):
	pass

class SwitchCameras(EventState):
	'''
	Get a random position based from an list of locations

	># camera_pick int          camera number pick start 		    
	># camera_place int 		camera number place start 
	># preGrasp_place string 	camera string pos place start 
	># preGrasp_pick string 	camera string pos place start
	#> camera_pick int          camera number pick end 		    
	#> camera_place int 		camera number place end 
	#> preGrasp_place string 	camera string pos place end 
	#> preGrasp_pick string 	camera string pos place end
	#> place_pose Poststamed 	poststamped




	<= done 			Given time has passed.


	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(SwitchCameras, self).__init__(outcomes = ['done'], output_keys = ['camera_pick','camera_place','preGrasp_place','preGrasp_pick',"place_pose"],input_keys=['camera_pick','camera_place','preGrasp_place','preGrasp_pick'])
		self.picks = 0
		#f = open("/home/" + user + "/misses.txt", "w")
		#f.write("0")

		rospy.loginfo("Init state: SwitchCameras state")




	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		userdata.place_pose = []
		tempCameraPick = userdata.camera_pick
		userdata.camera_pick = userdata.camera_place
		userdata.camera_place = tempCameraPick

		tempCameraPickGrasp = userdata.preGrasp_pick
		userdata.preGrasp_pick = userdata.preGrasp_place
		userdata.preGrasp_place = tempCameraPickGrasp
		#rospy.loginfo("camera pick= %i",userdata.camera_pick)
		#rospy.loginfo("camera pick= %s",userdata.preGrasp_pick)

		#f = open("/home/" + user + "/amount.txt", "r")
		#i = int(f.read())
		#pickPerCycle = i - self.picks
		#self.picks = i
		#rospy.loginfo("switchstate")
		#rospy.loginfo(pickPerCycle)
		#missesCycle = pickPerCycle - 4
		#f = open("/home/" + user + "/misses.txt", "r")
		#totalMisses = int(f.read())
		#rospy.loginfo(totalMisses)

		#f = open("/home/" + user + "/misses.txt", "w")
		#f.write(str(missesCycle + totalMisses))

		return 'done'
		pass
	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.
		
		pass

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass 

	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.
		pass


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
		
