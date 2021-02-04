#!/usr/bin/env python

import rospy

from magician_hardware.srv import isConnected, isConnectedRequest, isConnectedResponse

from flexbe_core import EventState, Logger


class CheckForConnection(EventState):
	'''
	check if connected
	--

	<= notConnected 					Not connected.


	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(CheckForConnection, self).__init__(outcomes = ['notConnected'])
		rospy.wait_for_service('/magician/isConnected')
		self.isConnected = rospy.ServiceProxy('magician/isConnected', isConnected)

		rospy.loginfo("Init state: CheckForConnection state")

		


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		response = self.isConnected()
		if not response.isConnected:
			return "notConnected"
	
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
		
