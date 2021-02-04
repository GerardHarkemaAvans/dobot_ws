#!/usr/bin/env python

import rospy

from magician_hardware.srv import reconnect, reconnectRequest, reconnectResponse
from magician_hardware.srv import isOnline, isOnlineRequest, isOnlineResponse
from std_msgs.msg import Bool

from flexbe_core import EventState, Logger


class Reconnect(EventState):
	'''
	reconnect
	--

	<= reconneced 					


	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Reconnect, self).__init__(outcomes = ['reconneced'])
		rospy.wait_for_service('/magician/reconnect')
		rospy.wait_for_service('/magician/isOnline')
		self.reconnect = rospy.ServiceProxy('magician/reconnect', reconnect)
		self.isOnline = rospy.ServiceProxy('magician/isOnline', isOnline)
		self.button_pushed = rospy.Publisher('button_pushed', Bool, queue_size=10)

		rospy.loginfo("Init state: Reconnect state")

		


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		response = self.isOnline()
		if response.isOnline and not self.online:
			try:
				self.online = True
				self._rate.sleep()
				self._start_time = rospy.get_rostime()
			except ROSInterruptException:
				rospy.logwarn('Skipped sleep.')

		if not self._start_time == 0 and not self.connected and self.online:
			elapsed = rospy.get_rostime() - self._start_time
			if elapsed.to_sec() > 10:
				reconRes = self.reconnect()
				if reconRes.succes:
					try:
						self.connected= True
						self._rate.sleep()
						self._connectedTime = rospy.get_rostime()
						rospy.loginfo("doe ik dit")
					except ROSInterruptException:
						rospy.logwarn('Skipped sleep.')
		rospy.loginfo("loop")
		if not self._connectedTime == 0:
			elapsed2 = rospy.get_rostime() - self._connectedTime
			if elapsed2.to_sec() > 5:
				return "reconneced"

			
	
	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		self._start_time = 0
		self._connectedTime = 0
		self.connected = False
		self.online = False
		self.button_pushed.publish(True)

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
		
