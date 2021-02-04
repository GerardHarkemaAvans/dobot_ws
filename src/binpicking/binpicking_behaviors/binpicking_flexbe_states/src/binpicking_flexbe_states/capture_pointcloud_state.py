#!/usr/bin/env python
import rospy

from binpicking_vision.srv import CapturePointcloud, CapturePointcloudRequest, CapturePointcloudResponse

from flexbe_core import EventState, Logger

from sensor_msgs.msg import PointCloud2
import copy

class CapturePointcloudState(EventState):
	'''
	Caputres a Pouncloud form the Realsensor

	># camera 	int 	The number of the camera
	#> pointcloud		PointCloud2		Pointcloud of the objects


	<= continue 					Given time has passed.
	<= failed 						Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(CapturePointcloudState, self).__init__(outcomes = ['continue', 'failed'], output_keys = ['pointcloud'],input_keys=['camera'])

		# Store state parameter for later use.

		# The constructor is called when building the state machine, not when actually starting the behavior.
		# Thus, we cannot save the starting time now and will do so later.
		rospy.wait_for_service('capture_pointcloud')
		# Create a service proxy.
		self.capture_pointcloud = rospy.ServiceProxy('capture_pointcloud', CapturePointcloud)

		rospy.loginfo("Init state: CapturePointcloudState state")


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		userdata.pointcloud = copy.deepcopy(self.service_response.pointcloud)
		return 'continue' # One of the outcomes declared above.


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.
		self.camera = userdata.camera
		rospy.loginfo("Picture has been made")
		try:
		# Call the service here.
			self.service_response = self.capture_pointcloud(userdata.camera)

		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		# In this example, we use this event to set the correct start time.
		pass # Nothing to do in this example.


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
