#!/usr/bin/env python
import rospy

from binpicking_vision.srv import CalculateObjectposeFromPointcloud, CalculateObjectposeFromPointcloudRequest, CalculateObjectposeFromPointcloudResponse

from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2

import copy


class CalculateObjectPoseState(EventState):
	'''
	Calcualtes the pose of a object from a pointcloud

	-- totalTries 		int				amount of retries

	># camera 			int 			The number of the camera
	># pointcloud		PointCloud2		Pointcloud of the objects
	#> object_pose		PoseStamped		Pose of the detected object
	#> hasFailed		boolean			booelaan wheter calculate has failed
	#> degrees			int				degrees to turn object


	<= continue 						Given time has passed.
	<= failed 							Example for a failure outcome.
	<= failedXtimes						Has failed x times  

	'''

	def __init__(self,totalTries):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(CalculateObjectPoseState, self).__init__(outcomes = ['continue', 'failed','failedXtimes'], input_keys = ['pointcloud','camera'], output_keys = ['object_pose','hasFailed','degrees'])
		# Store state parameter for later use.
		self._tries = 0
		self._totalTries = totalTries


		rospy.loginfo("Init state: CalculateObjectPoseState state")

	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.
		if not self.service_response.succes:
			userdata.hasFailed = True
			self._tries += 1
			if self._tries  == self._totalTries:
				self._tries = 0
				return 'failedXtimes'
			return 'failed' #When nothing is found aka empty
		userdata.object_pose = copy.deepcopy(self.service_response.object_pose)
		userdata.degrees = self.service_response.degrees
		userdata.hasFailed = False
		return 'continue' # One of the outcomes declared above.


	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		# The following code is just for illustrating how the behavior logger works.
		# Text logged by the behavior logger is sent to the operator and displayed in the GUI.
		self._camera = userdata.camera
		rospy.wait_for_service('calculate_object_pose')
		# Create a service proxy.
		self.calculate_object_pose = rospy.ServiceProxy('calculate_object_pose', CalculateObjectposeFromPointcloud)


		request = CalculateObjectposeFromPointcloudRequest()
		request.pointcloud =  copy.deepcopy(userdata.pointcloud)
		request.camera = self._camera
		try:
			# Call the service here.
			self.service_response = self.calculate_object_pose(request)
			rospy.loginfo(self.service_response.object_pose)

		except rospy.ServiceException, e:
			rospy.logerr("Service call failed")

		#pass # Nothing to do in this example.

	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass # Nothing to do in this example.


	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		pass # Nothing to do in this example.


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass # Nothing to do in this example.
