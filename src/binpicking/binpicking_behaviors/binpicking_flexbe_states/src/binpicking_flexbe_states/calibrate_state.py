#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from magician_hardware.srv import calibrate
from magician_hardware.srv import isDoneExecutingCommand


'''
Created on 20.11.2020
@author: Maarten Nieuwenhuize
'''

class CalibrateState(EventState):
	'''
	Implements a state that can calibrate the dobot magician.
	
	<= done					indicates that the suctioncup has turned off/on.
	<= failed 				Example for a failure outcome.
	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(CalibrateState, self).__init__(outcomes=['done',"failed"])
		#f = open("/home/binpicking/tries.log", "w") #For reseting file 

		rospy.loginfo("Init state: CalibrateState state")
		
	def execute(self, userdata):
		#Wait for 1 second
		elapsed = rospy.get_rostime() - self._start_time
		if elapsed.to_sec() > 1:
			rospy.wait_for_service('magician/isDoneExecutingCommand')
			try:
				isDoneExecutingCommandFunction = rospy.ServiceProxy('magician/isDoneExecutingCommand', isDoneExecutingCommand)
				resp = isDoneExecutingCommandFunction(self._index)
				if resp.isDoneExecutingCommand:
					return 'done'
			except rospy.ServiceException as e: 
				rospy.logerr("isDoneExecutingCommand service not found trying again...")
	
	def on_enter(self, userdata):
		rospy.loginfo("Starting calibrate state")
		rospy.wait_for_service('magician/calibrate')
		#Call calibrate 
		try:
			calibrateFunction = rospy.ServiceProxy('magician/calibrate', calibrate)
			resp1 = calibrateFunction()
			self._index = resp1.index
			#wait half a sec for the dobot to start moving
			try:
				self._rate.sleep()
				self._start_time = rospy.get_rostime()
			except ROSInterruptException:
				rospy.logwarn('Skipped sleep.')
		except rospy.ServiceException as e: 
			rospy.logerr("Calibrate service not found trying again...")
			return "failed"

		
