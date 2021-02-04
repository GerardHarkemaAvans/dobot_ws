#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from binpicking_flexbe_states.calibrate_state import CalibrateState
from binpicking_flexbe_states.if_camera_state import IfCamera
from binpicking_flexbe_states.move_to_predefined_pos_state import MoveToPredefinedPos
from binpicking_flexbe_states.suction_state import SuctionState
from binpicking_flexbe_states.move_to_pos_state import MoveToPos
from binpicking_flexbe_states.isMoving_state import IsMovingState
from binpicking_flexbe_states.turn_suction_cup_state import TurnSuctionCup
from binpicking_flexbe_states.get_place_position_state import GetPlacePosition
from binpicking_flexbe_states.calculate_object_pose_state import CalculateObjectPoseState
from binpicking_flexbe_states.switch_cameras_state import SwitchCameras
from binpicking_flexbe_states.capture_pointcloud_state import CapturePointcloudState
from binpicking_flexbe_states.idle_button_check_state import IdleButtonCheck
from flexbe_utility_states.wait_state import WaitState
from binpicking_flexbe_states.check_for_connection_state import CheckForConnection
from binpicking_flexbe_states.idle_state import Idle
from binpicking_flexbe_states.reconnect_state import Reconnect
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 12 2020
@author: Group 7
'''
class binpickingSM(Behavior):
	'''
	The behaviour for the demo. Picks object from the camera.
	'''


	def __init__(self):
		super(binpickingSM, self).__init__()
		self.name = 'binpicking'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 119 161 /idle check
		# Check for the connection of the dobot

		# O 813 182 /idle check
		# Main loop

		# O 57 27 
		# Calibrate the robot

		# O 418 374 
		# Reconnect the robot after the robot has been disconnected

		# O 926 15 
		# Idle state wait for every item to be ready and for the user to press the start button

		# O 510 222 
		# Turn the suction cup off

		# O 331 81 
		# Move to the home position

		# O 581 89 
		# Wait for the move to home to finish

		# O 805 172 
		# Main loop of item picking

		# O 357 59 /idle check
		# these states work in parralel

		# O 230 22 /idle check/Pick objects
		# Move to the place location and turn the suction cup off

		# O 39 337 /idle check/Pick objects
		# Switch the pick and place areas after al the items have been moved

		# O 460 121 /idle check/Pick objects
		# Make a picture of the pointcloud and move back to prePlace position. Also check if the button has been pressed

		# O 670 252 /idle check/Pick objects
		# Calculate the middlepoint of the highest object

		# O 802 440 /idle check/Pick objects
		# Get the location the item needs to be placed at

		# O 251 464 /idle check/Pick objects
		# Turn the suctioncup on and move to pickup the  object

		# O 12 302 /idle check/Pick objects/Move to place location and turn suction off
		# Ignore these 

		# O 26 35 /idle check/Pick objects/Move to place location and turn suction off
		# Move to the prePlace position

		# O 225 123 /idle check/Pick objects/Move to place location and turn suction off
		# Move to 5 cm above the place position

		# O 238 319 /idle check/Pick objects/Move to place location and turn suction off
		# Move to the place position

		# O 514 232 /idle check/Pick objects/Move to place location and turn suction off
		# Wait for the move to finish

		# O 533 431 /idle check/Pick objects/Move to place location and turn suction off
		# Turn the suction cup off

		# O 58 135 /idle check/Pick objects/Move back to preGrasp pick and capture pointcloud
		# Take 1 pointcloud from the camera. The robot has just placed an object

		# O 348 156 /idle check/Pick objects/Move back to preGrasp pick and capture pointcloud
		# Move up to the place location +5cm|n

		# O 325 312 /idle check/Pick objects/Move back to preGrasp pick and capture pointcloud
		# Move back to the prePlace location again



	def create(self):
		move_group = "magician_arm"
		move_group_prefix = ""
		offset = 0
		rotation = 0
		end_effctor = "magician_end_link"
		joint_names = ["magician_joint1","magician_joint2", "magician_joint3","magician_fake_joint1","magician_fake_joint2","magician_fake_joint3"]
		camera = 2
		camera_pick_start = 1
		camera_place_start = 2
		preGrasp_place_start = "grasp2"
		preGrasp_pick_start = "grasp1"
		degreesToTurn = 0
		# x:1002 y:371, x:158 y:373
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.turnOn = True
		_state_machine.userdata.captured_pointcloud = []
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.end_effctor = end_effctor
		_state_machine.userdata.offset = offset
		_state_machine.userdata.rotation = rotation
		_state_machine.userdata.shouldResetPlaceLoc = False
		_state_machine.userdata.place_pose = []
		_state_machine.userdata.camera_pick = camera_pick_start
		_state_machine.userdata.camera_place = camera_place_start
		_state_machine.userdata.preGrasp_place = preGrasp_place_start
		_state_machine.userdata.preGrasp_pick = preGrasp_pick_start
		_state_machine.userdata.degreesToTurn = degreesToTurn
		_state_machine.userdata.zeroDegrees = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:463, x:130 y:463
		_sm_move_to_object_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part_pose'])

		with _sm_move_to_object_0:
			# x:329 y:181
			OperatableStateMachine.add('MoveToPlace2',
										MoveToPos(offSetZ=-0.013),
										transitions={'continue': 'finished', 'invalidMove': 'failed', 'noPose': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalidMove': Autonomy.Off, 'noPose': Autonomy.Off},
										remapping={'pose': 'part_pose'})


		# x:30 y:463, x:130 y:463
		_sm_move_to_object_10_above_again_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part_pose'])

		with _sm_move_to_object_10_above_again_1:
			# x:265 y:132
			OperatableStateMachine.add('MoveToPlace2',
										MoveToPos(offSetZ=0.05),
										transitions={'continue': 'finished', 'invalidMove': 'failed', 'noPose': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalidMove': Autonomy.Off, 'noPose': Autonomy.Off},
										remapping={'pose': 'part_pose'})


		# x:30 y:463, x:130 y:463
		_sm_move_to_object_10_above_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part_pose'])

		with _sm_move_to_object_10_above_2:
			# x:109 y:140
			OperatableStateMachine.add('MoveToPlace2',
										MoveToPos(offSetZ=0.05),
										transitions={'continue': 'finished', 'invalidMove': 'failed', 'noPose': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalidMove': Autonomy.Off, 'noPose': Autonomy.Off},
										remapping={'pose': 'part_pose'})


		# x:30 y:463, x:130 y:463
		_sm_move_to_pick_pregrasp_again_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['camera_pick'])

		with _sm_move_to_pick_pregrasp_again_3:
			# x:212 y:21
			OperatableStateMachine.add('IfCamera0',
										IfCamera(),
										transitions={'one': 'move to preGrasp1', 'two': 'mote to preGrasp2'},
										autonomy={'one': Autonomy.Off, 'two': Autonomy.Off},
										remapping={'camera': 'camera_pick'})

			# x:372 y:141
			OperatableStateMachine.add('mote to preGrasp2',
										MoveToPredefinedPos(preDefPos="preGrasp2"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:24 y:146
			OperatableStateMachine.add('move to preGrasp1',
										MoveToPredefinedPos(preDefPos="preGrasp1"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})


		# x:30 y:463, x:130 y:463
		_sm_move_to_place_preplace_again_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['camera_pick'])

		with _sm_move_to_place_preplace_again_4:
			# x:266 y:26
			OperatableStateMachine.add('IfCamera0',
										IfCamera(),
										transitions={'one': 'mote to preGrasp2', 'two': 'move to preGrasp1'},
										autonomy={'one': Autonomy.Off, 'two': Autonomy.Off},
										remapping={'camera': 'camera_pick'})

			# x:484 y:153
			OperatableStateMachine.add('mote to preGrasp2',
										MoveToPredefinedPos(preDefPos="preGrasp2"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:15 y:150
			OperatableStateMachine.add('move to preGrasp1',
										MoveToPredefinedPos(preDefPos="preGrasp1"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})


		# x:30 y:463, x:130 y:463
		_sm_move_to_pick_pregrasp_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['camera_pick'])

		with _sm_move_to_pick_pregrasp_5:
			# x:30 y:29
			OperatableStateMachine.add('move to Home',
										MoveToPredefinedPos(preDefPos="home"),
										transitions={'continue': 'IfCamera', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:401 y:139
			OperatableStateMachine.add('mote to preGrasp2',
										MoveToPredefinedPos(preDefPos="preGrasp2"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:35 y:138
			OperatableStateMachine.add('move to preGrasp1',
										MoveToPredefinedPos(preDefPos="preGrasp1"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:212 y:21
			OperatableStateMachine.add('IfCamera',
										IfCamera(),
										transitions={'one': 'move to preGrasp1', 'two': 'mote to preGrasp2'},
										autonomy={'one': Autonomy.Off, 'two': Autonomy.Off},
										remapping={'camera': 'camera_pick'})


		# x:30 y:463, x:130 y:463
		_sm_move_to_place_location_and_5cm_again_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['place_pose'])

		with _sm_move_to_place_location_and_5cm_again_6:
			# x:262 y:128
			OperatableStateMachine.add('MoveToPlace2',
										MoveToPos(offSetZ=0.05),
										transitions={'continue': 'finished', 'invalidMove': 'failed', 'noPose': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalidMove': Autonomy.Off, 'noPose': Autonomy.Off},
										remapping={'pose': 'place_pose'})


		# x:30 y:463, x:130 y:463
		_sm_move_to_place_location_and_5cm_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['place_pose'])

		with _sm_move_to_place_location_and_5cm_7:
			# x:244 y:212
			OperatableStateMachine.add('MoveToPlace2',
										MoveToPos(offSetZ=0.05),
										transitions={'continue': 'finished', 'invalidMove': 'failed', 'noPose': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalidMove': Autonomy.Off, 'noPose': Autonomy.Off},
										remapping={'pose': 'place_pose'})


		# x:30 y:463, x:130 y:463
		_sm_move_to_place_location_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['place_pose'])

		with _sm_move_to_place_location_8:
			# x:80 y:149
			OperatableStateMachine.add('MoveToPlace2',
										MoveToPos(offSetZ=0),
										transitions={'continue': 'finished', 'invalidMove': 'failed', 'noPose': 'failed'},
										autonomy={'continue': Autonomy.Off, 'invalidMove': Autonomy.Off, 'noPose': Autonomy.Off},
										remapping={'pose': 'place_pose'})


		# x:30 y:463, x:130 y:463
		_sm_move_to_place_preplace_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['camera_pick'])

		with _sm_move_to_place_preplace_9:
			# x:266 y:26
			OperatableStateMachine.add('IfCamera0',
										IfCamera(),
										transitions={'one': 'mote to preGrasp2', 'two': 'move to preGrasp1'},
										autonomy={'one': Autonomy.Off, 'two': Autonomy.Off},
										remapping={'camera': 'camera_pick'})

			# x:15 y:150
			OperatableStateMachine.add('move to preGrasp1',
										MoveToPredefinedPos(preDefPos="preGrasp1"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:484 y:153
			OperatableStateMachine.add('mote to preGrasp2',
										MoveToPredefinedPos(preDefPos="preGrasp2"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})


		# x:1092 y:742, x:130 y:463
		_sm_suc_on_and_move_to_grab_and_then_to_home_10 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['part_pose', 'camera_pick', 'degreesToTurn'])

		with _sm_suc_on_and_move_to_grab_and_then_to_home_10:
			# x:75 y:40
			OperatableStateMachine.add('SucOn',
										SuctionState(turnOn=True),
										transitions={'done': 'Move To Object 10 above'},
										autonomy={'done': Autonomy.Off})

			# x:735 y:444
			OperatableStateMachine.add('Move To pick preGrasp again',
										_sm_move_to_pick_pregrasp_again_3,
										transitions={'finished': 'move to Home', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick'})

			# x:345 y:39
			OperatableStateMachine.add('Move To Object 10 above',
										_sm_move_to_object_10_above_2,
										transitions={'finished': 'Move To Object', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part_pose': 'part_pose'})

			# x:732 y:350
			OperatableStateMachine.add('Move To Object 10 above again',
										_sm_move_to_object_10_above_again_1,
										transitions={'finished': 'Move To pick preGrasp again', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part_pose': 'part_pose'})

			# x:1016 y:227
			OperatableStateMachine.add('WaitForGrabMove',
										IsMovingState(),
										transitions={'done': 'WaitForVacuum'},
										autonomy={'done': Autonomy.Off})

			# x:746 y:739
			OperatableStateMachine.add('move to Home',
										MoveToPredefinedPos(preDefPos="home"),
										transitions={'continue': 'finished', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:1018 y:117
			OperatableStateMachine.add('Move To Object',
										_sm_move_to_object_0,
										transitions={'finished': 'WaitForGrabMove', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part_pose': 'part_pose'})

			# x:784 y:231
			OperatableStateMachine.add('WaitForVacuum',
										WaitState(wait_time=0.15),
										transitions={'done': 'Move To Object 10 above again'},
										autonomy={'done': Autonomy.Off})

			# x:1014 y:30
			OperatableStateMachine.add('TurnDegrees',
										TurnSuctionCup(),
										transitions={'continue': 'Move To Object', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'degrees': 'degreesToTurn'})

			# x:601 y:41
			OperatableStateMachine.add('WaitForTurn',
										IsMovingState(),
										transitions={'done': 'TurnDegrees'},
										autonomy={'done': Autonomy.Off})


		# x:1398 y:245, x:433 y:722, x:766 y:89
		_sm_move_back_to_pregrasp_pick_and_capture_pointcloud_11 = OperatableStateMachine(outcomes=['finished', 'failed', 'pressed'], input_keys=['camera_pick', 'place_pose', 'zeroDegrees'], output_keys=['captured_pointcloud'])

		with _sm_move_back_to_pregrasp_pick_and_capture_pointcloud_11:
			# x:106 y:182
			OperatableStateMachine.add('CapturePointcloud',
										CapturePointcloudState(),
										transitions={'continue': 'Move To Place location and 5cm again', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'camera': 'camera_pick', 'pointcloud': 'captured_pointcloud'})

			# x:351 y:184
			OperatableStateMachine.add('Move To Place location and 5cm again',
										_sm_move_to_place_location_and_5cm_again_6,
										transitions={'finished': 'Move to place prePlace again', 'failed': 'ButtonCheck'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'place_pose': 'place_pose'})

			# x:718 y:238
			OperatableStateMachine.add('ButtonCheck',
										IdleButtonCheck(),
										transitions={'pressed': 'pressed', 'notPressed': 'Move to pick preGrasp'},
										autonomy={'pressed': Autonomy.Off, 'notPressed': Autonomy.Off})

			# x:947 y:234
			OperatableStateMachine.add('Move to pick preGrasp',
										_sm_move_to_pick_pregrasp_5,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick'})

			# x:367 y:337
			OperatableStateMachine.add('Move to place prePlace again',
										_sm_move_to_place_preplace_again_4,
										transitions={'finished': 'ButtonCheck', 'failed': 'ButtonCheck'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick'})


		# x:254 y:485, x:60 y:223
		_sm_move_to_place_location_and_turn_suction_off_12 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['camera_pick', 'place_pose', 'degreesToTurn', 'zeroDegrees'])

		with _sm_move_to_place_location_and_turn_suction_off_12:
			# x:23 y:63
			OperatableStateMachine.add('Move to place prePlace',
										_sm_move_to_place_preplace_9,
										transitions={'finished': 'Move To Place location and 5cm', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick'})

			# x:532 y:456
			OperatableStateMachine.add('SucOff',
										SuctionState(turnOn=False),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:237 y:350
			OperatableStateMachine.add('Move To Place location',
										_sm_move_to_place_location_8,
										transitions={'finished': 'WaitForGrapMove', 'failed': 'WaitForGrapMove'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'place_pose': 'place_pose'})

			# x:227 y:149
			OperatableStateMachine.add('Move To Place location and 5cm',
										_sm_move_to_place_location_and_5cm_7,
										transitions={'finished': 'Move To Place location', 'failed': 'WaitForGrapMove'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'place_pose': 'place_pose'})

			# x:514 y:254
			OperatableStateMachine.add('WaitForGrapMove',
										IsMovingState(),
										transitions={'done': 'SucOff'},
										autonomy={'done': Autonomy.Off})

			# x:11 y:330
			OperatableStateMachine.add('TurnDegrees',
										TurnSuctionCup(),
										transitions={'continue': 'Move To Place location', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'degrees': 'zeroDegrees'})

			# x:14 y:471
			OperatableStateMachine.add('WaitForTurn',
										IsMovingState(),
										transitions={'done': 'TurnDegrees'},
										autonomy={'done': Autonomy.Off})


		# x:1280 y:176, x:410 y:305
		_sm_pick_objects_13 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['camera_pick', 'camera_place', 'preGrasp_place', 'part_pose', 'place_pose', 'preGrasp_pick', 'degreesToTurn', 'zeroDegrees'])

		with _sm_pick_objects_13:
			# x:231 y:55
			OperatableStateMachine.add('Move to place location and turn suction off',
										_sm_move_to_place_location_and_turn_suction_off_12,
										transitions={'finished': 'Move back to preGrasp pick and capture pointcloud', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick', 'place_pose': 'place_pose', 'degreesToTurn': 'degreesToTurn', 'zeroDegrees': 'zeroDegrees'})

			# x:802 y:388
			OperatableStateMachine.add('GetPlaceLocation',
										GetPlacePosition(),
										transitions={'done': 'Suc on and move to grab and then to home', 'hasReset': 'SwitchCameras', 'full': 'finished'},
										autonomy={'done': Autonomy.Off, 'hasReset': Autonomy.Off, 'full': Autonomy.Off},
										remapping={'camera': 'camera_place', 'shouldReset': 'shouldResetPlaceLoc', 'pick_pose': 'part_pose', 'pose': 'place_pose'})

			# x:671 y:286
			OperatableStateMachine.add('CalculateObjectPoseState',
										CalculateObjectPoseState(totalTries=2),
										transitions={'continue': 'GetPlaceLocation', 'failed': 'GetPlaceLocation', 'failedXtimes': 'GetPlaceLocation'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off, 'failedXtimes': Autonomy.Off},
										remapping={'pointcloud': 'captured_pointcloud', 'camera': 'camera_pick', 'object_pose': 'part_pose', 'hasFailed': 'shouldResetPlaceLoc', 'degrees': 'degreesToTurn'})

			# x:135 y:385
			OperatableStateMachine.add('SwitchCameras',
										SwitchCameras(),
										transitions={'done': 'Move to place location and turn suction off'},
										autonomy={'done': Autonomy.Off},
										remapping={'camera_pick': 'camera_pick', 'camera_place': 'camera_place', 'preGrasp_place': 'preGrasp_place', 'preGrasp_pick': 'preGrasp_pick', 'place_pose': 'place_pose'})

			# x:460 y:166
			OperatableStateMachine.add('Move back to preGrasp pick and capture pointcloud',
										_sm_move_back_to_pregrasp_pick_and_capture_pointcloud_11,
										transitions={'finished': 'CalculateObjectPoseState', 'failed': 'failed', 'pressed': 'finished'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'pressed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick', 'place_pose': 'place_pose', 'zeroDegrees': 'zeroDegrees', 'captured_pointcloud': 'captured_pointcloud'})

			# x:250 y:506
			OperatableStateMachine.add('Suc on and move to grab and then to home',
										_sm_suc_on_and_move_to_grab_and_then_to_home_10,
										transitions={'finished': 'Move to place location and turn suction off', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'part_pose': 'part_pose', 'camera_pick': 'camera_pick', 'degreesToTurn': 'degreesToTurn'})


		# x:600 y:327, x:408 y:203, x:794 y:326, x:346 y:676, x:511 y:682
		_sm_idle_check_14 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['camera_pick', 'camera_place', 'preGrasp_place', 'part_pose', 'preGrasp_pick', 'place_pose', 'degreesToTurn', 'zeroDegrees'], conditions=[
										('finished', [('Pick objects', 'finished')]),
										('finished', [('Pick objects', 'failed')]),
										('failed', [('ChecforConnection', 'notConnected')])
										])

		with _sm_idle_check_14:
			# x:684 y:170
			OperatableStateMachine.add('Pick objects',
										_sm_pick_objects_13,
										transitions={'finished': 'finished', 'failed': 'finished'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick', 'camera_place': 'camera_place', 'preGrasp_place': 'preGrasp_place', 'part_pose': 'part_pose', 'place_pose': 'place_pose', 'preGrasp_pick': 'preGrasp_pick', 'degreesToTurn': 'degreesToTurn', 'zeroDegrees': 'zeroDegrees'})

			# x:121 y:193
			OperatableStateMachine.add('ChecforConnection',
										CheckForConnection(),
										transitions={'notConnected': 'failed'},
										autonomy={'notConnected': Autonomy.Off})



		with _state_machine:
			# x:56 y:59
			OperatableStateMachine.add('calibrate',
										CalibrateState(),
										transitions={'done': 'move to Home', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:803 y:203
			OperatableStateMachine.add('idle check',
										_sm_idle_check_14,
										transitions={'finished': 'SucOff_2', 'failed': 'Reconnect'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'camera_pick': 'camera_pick', 'camera_place': 'camera_place', 'preGrasp_place': 'preGrasp_place', 'part_pose': 'part_pose', 'preGrasp_pick': 'preGrasp_pick', 'place_pose': 'place_pose', 'degreesToTurn': 'degreesToTurn', 'zeroDegrees': 'zeroDegrees'})

			# x:786 y:9
			OperatableStateMachine.add('idle',
										Idle(),
										transitions={'pressed': 'idle check', 'calibrate': 'calibrate'},
										autonomy={'pressed': Autonomy.Off, 'calibrate': Autonomy.Off})

			# x:584 y:120
			OperatableStateMachine.add('WaitForHome',
										IsMovingState(),
										transitions={'done': 'idle'},
										autonomy={'done': Autonomy.Off})

			# x:482 y:419
			OperatableStateMachine.add('Reconnect',
										Reconnect(),
										transitions={'reconneced': 'calibrate'},
										autonomy={'reconneced': Autonomy.Off})

			# x:333 y:102
			OperatableStateMachine.add('move to Home',
										MoveToPredefinedPos(preDefPos="home"),
										transitions={'continue': 'WaitForHome', 'wrongParams': 'failed', 'invalidMove': 'failed'},
										autonomy={'continue': Autonomy.Off, 'wrongParams': Autonomy.Off, 'invalidMove': Autonomy.Off})

			# x:510 y:250
			OperatableStateMachine.add('SucOff_2',
										SuctionState(turnOn=False),
										transitions={'done': 'move to Home'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
