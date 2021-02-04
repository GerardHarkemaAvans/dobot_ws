#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from magician_hardware.srv import Suction

'''
Created on 13.11.2020
@author: Maarten Nieuwenhuize
'''

class SuctionState(EventState):
	'''
	Implements a state that can turn the suction cup of the dobot on and off.
	
	-- turnOn  bool	boolean that indicates the state of the suction cup (if true the suction cup turns on).
	<= done						indicates that the suctioncup has turned off/on.
	'''


	def __init__(self, turnOn):
		'''
		Constructor
		'''
		super(SuctionState, self).__init__(outcomes=['done'])
		
		self._turnOn = turnOn
		#f = open("/home/binpicking/amount.txt", "w")
		#f.write("0")
		rospy.wait_for_service('magician/suction')
		self.suction = rospy.ServiceProxy('magician/suction', Suction)
		rospy.loginfo("Init state: suctionState")

		
	def execute(self, userdata):
		if not self.timeHasStarted:
			try:
				#if self._turnOn:
					#f = open("/home/binpicking/amount.txt", "r")
					#i = int(f.read())
					#f = open("/home/binpicking/amount.txt", "w")
					#f.write(str(i + 1))
					#f.close()
				resp1 = self.suction(self._turnOn)
				try:
					self.timeHasStarted = True
					self._rate.sleep()
					self._start_time = rospy.get_rostime()
				except ROSInterruptException:
					rospy.logwarn('Skipped sleep.')
			except rospy.ServiceException as e: 
				print("Service call failed: %s"%e)
		if self.timeHasStarted and not self._turnOn:
			elapsed = rospy.get_rostime() - self._start_time
			if elapsed.to_sec() > 0.1:
				return 'done'
		elif self.timeHasStarted and self._turnOn:
			return 'done'

	def on_enter(self, userdata):		
		self._start_time = 0
		self.timeHasStarted = False
		
