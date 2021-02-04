#!/usr/bin/env python

import rospy
import math
import random
from geometry_msgs.msg import PoseStamped

from flexbe_core import EventState, Logger

class test( object ):
    pass

class GetPlacePosition(EventState):
	'''
	Get a random position based from an list of locations

	># camera int				camera number 
	># shouldReset	boolean	 	wheter to reset the list of placed parts
	># pick_pose PoseStamped 	pose of the picked up opbject
	#> pose PoseStamped 		where to place the object

	<= done 			Given time has passed.
	<= hasReset 			When the list of placed object resets
	<= full 				When no places of the list are left

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(GetPlacePosition, self).__init__(outcomes = ['done','hasReset','full'], output_keys = ['pose'],input_keys=['camera','shouldReset','pick_pose'])

		#Init locations
		pos1 = [0.23,-0.06,0.1]
		pos2 = [0.23,0.05,0.1]
		pos3 = [0.23,0.17,0.1]
		pos4 = [0.12,0.16,0.1]
		pos5 = [0.12,0.27,0.1]
		

		pos7 = [-0.23,-0.06,0.1]
		pos8 = [-0.23,0.05,0.1]
		pos9 = [-0.23,0.17,0.1]
		pos10 = [-0.12,0.16,0.1]
		pos11 = [-0.12,0.27,0.1]
		
	


		self._locations_for_camera_1_as_place = [pos1,pos2,pos3,pos4,pos5]
		self._locations_for_camera_2_as_place = [pos7,pos8,pos9,pos10,pos11]
		self._placedIndexes = []

		rospy.loginfo("Init state: GetPlacePosition state")




	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		#If reset has been done reset al the placed objects
		rospy.loginfo(userdata.shouldReset)
		if userdata.shouldReset:
			self._placedIndexes = []
			return 'hasReset'
		
		#if the lenght of locations is full then its full 
		if (len(self._locations_for_camera_1_as_place) == len(self._placedIndexes) and userdata.camera == 1) or len(self._locations_for_camera_2_as_place) == len(self._placedIndexes) :
			self._placedIndexes = []
			return 'full'

		#place the object
		index = 0
		if userdata.camera == 1:
			index = random.randint(0,len(self._locations_for_camera_1_as_place) - 1)
		else:
			index = random.randint(0,len(self._locations_for_camera_2_as_place) - 1)

		if index not in self._placedIndexes:
			pose = PoseStamped()
			if userdata.camera == 1:
				pose.pose.position.x = self._locations_for_camera_1_as_place[index][0]
				pose.pose.position.y = self._locations_for_camera_1_as_place[index][1]
				pose.pose.position.z = userdata.pick_pose.pose.position.z
			else:
				pose.pose.position.x = self._locations_for_camera_2_as_place[index][0]
				pose.pose.position.y = self._locations_for_camera_2_as_place[index][1]
				pose.pose.position.z = userdata.pick_pose.pose.position.z
			userdata.pose = pose
			rospy.loginfo(pose)
			rospy.loginfo(index)
			self._placedIndexes.append(index)
			return 'done'

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
		
