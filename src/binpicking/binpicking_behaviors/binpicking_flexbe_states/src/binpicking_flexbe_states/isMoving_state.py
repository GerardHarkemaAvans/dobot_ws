#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from magician_hardware.srv import isMoving

'''
Created on 20.11.2020
@author: Maarten Nieuwenhuize
'''

class IsMovingState(EventState):
	'''
	Implements a state that stops when the dobot is not moving.
	
	<= done						indicates that the dobot has stopped moving.
	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(IsMovingState, self).__init__(outcomes=['done'])

		rospy.loginfo("Init state: IsMovingState state")

		
		
	def execute(self, userdata):
		rospy.wait_for_service('magician/isMoving')
		elapsed = rospy.get_rostime() - self._start_time
		if elapsed.to_sec() > 0.5:
			try:
				isMovingFunction = rospy.ServiceProxy('magician/isMoving', isMoving)
				resp = isMovingFunction()
				if not resp.isMoving:
					self._numberOfchecks += 1
				else:
					self._numberOfchecks = 0

				if self._numberOfchecks == 4:
					self._numberOfchecks = 0
					return 'done'

			except rospy.ServiceException as e: 
				rospy.logerr("IsMoving service not found trying again...")

	def on_enter(self, userdata):
		self._numberOfchecks = 0
		rospy.loginfo("Starting calbirate state")
		try:
			self._rate.sleep()
			self._start_time = rospy.get_rostime()
		except ROSInterruptException:
			rospy.logwarn('Skipped sleep.')
	

