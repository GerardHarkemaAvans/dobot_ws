#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from datetime import datetime

import math

import getpass

user = getpass.getuser()


from flexbe_core import EventState, Logger


class PoseToMoveit(EventState):
	'''
	This state moves the robot arm to a position- it may fail so try again

	-- amountOfTries 	int 	The number of amount to try the move
	-- offSetZ 	float 	The offset of the z axis in meters
	># pose			PoseStamped	pose of the part to pick

	<= continue 			Given time has passed.
	<= failed 				Failure outcome.

	'''

	def __init__(self, amountOfTries,offSetZ):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(PoseToMoveit, self).__init__(outcomes = ['continue', 'failed'],input_keys = ['pose'])

		# Store camera parameter for later use.
		self._amountOfTries = amountOfTries
		self._offSetZ = offSetZ

		rospy.loginfo("Init state: PoseToMoveit state")


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		moveit_commander.roscpp_initialize(sys.argv)
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		group_name = "magician_arm"
		group = moveit_commander.MoveGroupCommander(group_name)
		
		rospy.loginfo(userdata.pose)
		if userdata.pose == []:
			return "continue"
		pos = userdata.pose.pose.position

		rospy.loginfo(pos)

		#check for if y == 0 (cant devide by 0)
		if pos.y == 0:
			pos.y = 0.0001

		#check if x is negative
		isNeg = 0
		if pos.x < 0 :
			isNeg = -1
		else:
			isNeg = 1

		#calculate the angle
		alpha = math.atan(isNeg * abs(pos.x / pos.y))
		if pos.x < 0 and pos.y < 0:
			alpha = -math.pi - alpha
		if pos.x > 0 and pos.y < 0:
			alpha = math.pi - alpha

		#calculate the length of arm end effec point 
		e = math.sqrt(pos.x ** 2 + pos.y ** 2) - 0.06
		rospy.loginfo("Lenght of e is %f",e)

		#calculate the new x and y values for the endeffector point of the robot
		x1 = math.sin(alpha) * e
		y1 = math.cos(alpha) * e
		rospy.loginfo("x1=%f y1=%f",x1,y1)

		position = [x1,y1,pos.z + self._offSetZ +0.075]
		group.set_position_target(position,"magician_end_link")

		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()

		if not plan:
			rospy.loginfo("PoseToMoveItState: %s tries remaining",self._amountOfTries - self._tries)
			self._tries+= 1
		else:
			#f = open("/home/" + user + "/tries.log", "a")
			#toWrite = "has succeded with tries " + str(self._tries) + " at " + str(datetime.now()) + '\n'
			#f.write(toWrite)
			#f.close()
			return "continue"
		if self._amountOfTries - self._tries == 0 :
			rospy.loginfo("PoseToMoveItState failed to try the move")
			#f = open("/home/" + user + "/tries.log", "a")
			#toWrite = "has failed with tries " + str(self._tries) + " at " + str(datetime.now()) + '\n'
			#f.write(toWrite)
			#f.close()
			return "continue"
			return "failed"

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.
		self._tries = 0
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
		
