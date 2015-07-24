#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_atlas_vehicle_checkout')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from vigir_flexbe_states.tilt_head_state import TiltHeadState
from vigir_flexbe_states.finger_configuration_state import FingerConfigurationState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

from flor_control_msgs.msg import FlorRobotStatus
from flexbe_core.proxy import ProxySubscriberCached

# [/MANUAL_IMPORT]


'''
Created on Sun May 24 2015
@author: Spyros Maniatopoulos
'''
class ATLASvehiclecheckoutSM(Behavior):
	'''
	A behavior for helping with getting ATLAS inside the vehicle and preparing to drive (Task #1)
	'''


	def __init__(self):
		super(ATLASvehiclecheckoutSM, self).__init__()
		self.name = 'ATLAS vehicle checkout'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:821 y:66, x:331 y:203
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.both = 'same'
		_state_machine.userdata.left = 'left'
		_state_machine.userdata.right = 'right'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:100 y:355, x:321 y:250
		_sm_prep_right_arm_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side'])

		with _sm_prep_right_arm_0:
			# x:61 y:89
			OperatableStateMachine.add('Go_to_Camera_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CAR_DRIVE_CAMERA_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Open_Fingers', 'failed': 'Go_to_Camera_Pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:65 y:228
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type='robotiq', configuration=0.0),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})


		# x:329 y:513, x:476 y:175
		_sm_prep_left_arm_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side'])

		with _sm_prep_left_arm_1:
			# x:79 y:59
			OperatableStateMachine.add('Look_Down',
										TiltHeadState(desired_tilt=TiltHeadState.DOWN_60),
										transitions={'done': 'Go_to_Pre_Drive', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High})

			# x:60 y:166
			OperatableStateMachine.add('Go_to_Pre_Drive',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CAR_PREDRIVE_LARM_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Open_Fingers', 'failed': 'Go_to_Pre_Drive'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:61 y:397
			OperatableStateMachine.add('Go_to_Drive',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CAR_DRIVE_LARM_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Notify_Field_Team', 'failed': 'Go_to_Drive'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:65 y:283
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type='robotiq', configuration=0.0),
										transitions={'done': 'Go_to_Drive', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})

			# x:71 y:504
			OperatableStateMachine.add('Notify_Field_Team',
										LogState(text='Ask field team to strap ATLAS', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})


		# x:409 y:455
		_sm_prep_car_entry_2 = OperatableStateMachine(outcomes=['finished'], input_keys=['both', 'left', 'right'])

		with _sm_prep_car_entry_2:
			# x:60 y:65
			OperatableStateMachine.add('Car_Entry_Arms',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CAR_ENTRY_ARMS_POSE, vel_scaling=0.3, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Car_Entry_Forearms', 'failed': 'Car_Entry_Arms'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'both'})

			# x:60 y:217
			OperatableStateMachine.add('Car_Entry_Forearms',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CAR_ENTRY_FORE_POSE, vel_scaling=0.5, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Car_Entry_Left_Leg', 'failed': 'Car_Entry_Forearms'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'both'})

			# x:60 y:351
			OperatableStateMachine.add('Car_Entry_Left_Leg',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CAR_ENTRY_LEGS_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Car_Entry_Right_Leg', 'failed': 'Car_Entry_Left_Leg'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'left'})

			# x:357 y:351
			OperatableStateMachine.add('Car_Entry_Right_Leg',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CAR_ENTRY_LEGS_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'finished', 'failed': 'Car_Entry_Right_Leg'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'side': 'right'})



		with _state_machine:
			# x:39 y:63
			OperatableStateMachine.add('Start_in_FREEZE_HP',
										ChangeControlModeActionState(target_mode='freeze_high_pressure'),
										transitions={'changed': 'Go_to_VEHICLE_ENTRY', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Full, 'failed': Autonomy.High})

			# x:51 y:446
			OperatableStateMachine.add('Prep_Car_Entry',
										_sm_prep_car_entry_2,
										transitions={'finished': 'Go_to_FREEZE_HP'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'both': 'both', 'left': 'left', 'right': 'right'})

			# x:278 y:451
			OperatableStateMachine.add('Go_to_FREEZE_HP',
										ChangeControlModeActionState(target_mode='freeze_high_pressure'),
										transitions={'changed': 'Confirm_All_Clear', 'failed': 'failed'},
										autonomy={'changed': Autonomy.High, 'failed': Autonomy.High})

			# x:547 y:226
			OperatableStateMachine.add('Prep_Left_Arm',
										_sm_prep_left_arm_1,
										transitions={'finished': 'Prep_Right_Arm', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'left'})

			# x:523 y:16
			OperatableStateMachine.add('Go_to_VEHICLE_DRIVE',
										ChangeControlModeActionState(target_mode='vehicle_drive'),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:515 y:347
			OperatableStateMachine.add('Back_to_VEHICLE_MANIPULATE',
										ChangeControlModeActionState(target_mode='vehicle_manipulate'),
										transitions={'changed': 'Prep_Left_Arm', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:550 y:449
			OperatableStateMachine.add('Confirm_All_Clear',
										LogState(text='Confirm that all personnel is clear!', severity=Logger.REPORT_HINT),
										transitions={'done': 'Back_to_VEHICLE_MANIPULATE'},
										autonomy={'done': Autonomy.Full})

			# x:39 y:252
			OperatableStateMachine.add('Go_to_VEHICLE_ENTRY',
										ChangeControlModeActionState(target_mode='vehicle_entry'),
										transitions={'changed': 'Prep_Car_Entry', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:543 y:101
			OperatableStateMachine.add('Prep_Right_Arm',
										_sm_prep_right_arm_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'right'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
