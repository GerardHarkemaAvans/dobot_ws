#!/usr/bin/env python

import rospy
import math
import random
from geometry_msgs.msg import PoseStamped

from magician_hardware.srv import moveToPosition, moveToPositionRequest, moveToPositionResponse
from geometry_msgs.msg import Point

from flexbe_core import EventState, Logger


class MoveToPredefinedPos(EventState):
	'''
	move to preDefinedPosition

	--preDefPos string					The predefined postition

	--

	<= continue 						Given time has passed.
	<= wrongParams 						the wrong params have been failed in
	<= invalidMove 						the move is not possible


	'''

	def __init__(self,preDefPos):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(MoveToPredefinedPos, self).__init__(outcomes = ['continue','wrongParams','invalidMove'])
		rospy.wait_for_service('/magician/moveToPosition')
		self.move_to_position = rospy.ServiceProxy('magician/moveToPosition', moveToPosition)

		self._preDefPos = preDefPos

		self.home = Point()
		self.home.x = 0
		self.home.y = 0.25
		self.home.z = 0.2

		self.preGrasp1 = Point()
		self.preGrasp1.x = 0.24
		self.preGrasp1.y = 0.13
		self.preGrasp1.z = 0.2

		self.preGrasp2 = Point()
		self.preGrasp2.x = -0.24
		self.preGrasp2.y = 0.13
		self.preGrasp2.z = 0.2
		
		rospy.loginfo("Init state: MoveToPredefinedPos state")




	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		switcher = {
			"home":self.home,
			"preGrasp1":self.preGrasp1,
			"preGrasp2": self.preGrasp2,
    	}
		rospy.loginfo(self._preDefPos)
		pos = switcher.get(self._preDefPos, "ERROR")
		if pos == "ERROR":
			return 'wrongParams'

		response = self.move_to_position(pos)
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
		
