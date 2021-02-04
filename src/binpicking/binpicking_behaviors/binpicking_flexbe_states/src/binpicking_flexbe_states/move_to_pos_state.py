#!/usr/bin/env python

import rospy
import math
import random
from geometry_msgs.msg import PoseStamped

from magician_hardware.srv import moveToPosition, moveToPositionRequest, moveToPositionResponse
from geometry_msgs.msg import Point

from flexbe_core import EventState, Logger


class MoveToPos(EventState):
	'''
	move to preDefinedPosition

	-- offSetZ 	float 					The offset of the z axis in meters

	># pose PoseStamped					The pose to move to

	--

	<= continue 						Given time has passed.
	<= invalidMove 						the move is not possible
	<= noPose 							No pose 


	'''

	def __init__(self,offSetZ):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(MoveToPos, self).__init__(outcomes = ['continue','invalidMove','noPose'],input_keys=['pose'])
		rospy.wait_for_service('/magician/moveToPosition')
		self.move_to_position = rospy.ServiceProxy('magician/moveToPosition', moveToPosition)
		self._offSetZ = offSetZ
		
		rospy.loginfo("Init state: MoveToPos state")


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		if userdata.pose == []:
			return "noPose"
		pos = userdata.pose.pose.position
		if pos.z > 0.12:
			return "invalidMove"
		pos.z = pos.z + self._offSetZ
		
		response = self.move_to_position(pos)
		pos.z = pos.z - self._offSetZ
		if response.succes:
			return "continue"
		else:
			return "invalidMove"
		
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
		
