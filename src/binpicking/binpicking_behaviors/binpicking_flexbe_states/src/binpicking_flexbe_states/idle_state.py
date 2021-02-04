#!/usr/bin/env python

import rospy

from binpicking_arduino.srv import isButtonPressed , isButtonPressedRequest ,isButtonPressedResponse
from binpicking_arduino.srv import setBlinking, setBlinkingRequest ,setBlinkingResponse
from binpicking_arduino.srv import isCalibrating, isCalibratingRequest ,isCalibratingResponse
from flexbe_core import EventState, Logger
from std_msgs.msg import Bool, Empty ,String


class Idle(EventState):
	'''
	state to check the button 

	<= pressed 							the button has been pressed
	<= calibrate						should calibrate

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Idle, self).__init__(outcomes = ['pressed','calibrate'])
		rospy.wait_for_service('binpicking/button')
		rospy.wait_for_service('binpicking/blink')
		rospy.wait_for_service('binpicking/calibrate')
		self.buttonCheck = rospy.ServiceProxy('binpicking/button', isButtonPressed)
		self.blink = rospy.ServiceProxy('binpicking/blink', setBlinking)
		self.isCalibrate = rospy.ServiceProxy('binpicking/calibrate', isCalibrating)
		self.calibrate = rospy.Publisher('calibrate', Bool, queue_size=10)
		self.button_pushed = rospy.Publisher('button_pushed', Bool, queue_size=10)

		rospy.loginfo("Init state: Idle state")


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		res = self.buttonCheck()
		if res.isPressed:
			return "pressed"

		resC = self.isCalibrate()
		if resC.isCalibratingPressed:
			return 'calibrate'
		
	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.\
		res = self.buttonCheck()
		if res.isPressed:
			self.button_pushed.publish(True)

		#req = setBlinkingRequest()
		#req.blink = True
		#self.blink(req)		
		pass

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.
		
		
		req = setBlinkingRequest()
		req.blink = False
		self.blink(req)
		self.calibrate.publish(False)
		pass 

	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.
		pass


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass
		
