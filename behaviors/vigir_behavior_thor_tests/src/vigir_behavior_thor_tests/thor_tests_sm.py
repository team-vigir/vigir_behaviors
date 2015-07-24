#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_thor_tests')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 09 2015
@author: Philipp Schillinger
'''
class THORTestsSM(Behavior):
	'''
	Testing states for THOR.
	'''


	def __init__(self):
		super(THORTestsSM, self).__init__()
		self.name = 'THOR Tests'

		# parameters of this behavior
		self.add_parameter('planning_group', 'l_arm_group')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		l_random = [-2.47557862791, -1.21674644795, -0.761510070214, -0.219080422969, -2.68076430953, 0.189685935246, -3.8825693651]
		r_random = [1.10260663602, 2.67164332545, 3.1286914453, 1.52042152218, -1.94191336135, 0.805351035619, -3.64437671794]
		# x:33 y:340, x:435 y:356
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joint_values = [0] * 7
		_state_machine.userdata.l_random = l_random
		_state_machine.userdata.r_random = r_random
		_state_machine.userdata.random = l_random if self.planning_group == "l_arm_group" else r_random
		_state_machine.userdata.hand_side = 'left'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:151 y:30
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Go_To_Stand', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:65 y:513
			OperatableStateMachine.add('Turn_Torso_Left_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_LEFT_POSE, vel_scaling=0.3, ignore_collisions=True, link_paddings={}),
										transitions={'done': 'Go_To_Open_Door_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'side': 'hand_side'})

			# x:312 y:565
			OperatableStateMachine.add('Go_To_Open_Door_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_OPEN_POSE_STRAIGHT, vel_scaling=0.1, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Back_To_Door_Ready_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'side': 'hand_side'})

			# x:647 y:328
			OperatableStateMachine.add('Go_To_Final_Stand',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Ask_If_Open', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'side': 'hand_side'})

			# x:622 y:189
			OperatableStateMachine.add('Ask_If_Open',
										OperatorDecisionState(outcomes=['open','push_again'], hint='Is the door open?', suggestion='open'),
										transitions={'open': 'finished', 'push_again': 'Log_Try_Push_Again'},
										autonomy={'open': Autonomy.High, 'push_again': Autonomy.Full})

			# x:355 y:189
			OperatableStateMachine.add('Log_Try_Push_Again',
										LogState(text='Move robot closer to door', severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_To_Door_Ready_Pose'},
										autonomy={'done': Autonomy.Full})

			# x:623 y:452
			OperatableStateMachine.add('Turn_Torso_Center_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_CENTER_POSE, vel_scaling=0.3, ignore_collisions=True, link_paddings={}),
										transitions={'done': 'Go_To_Final_Stand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'side': 'hand_side'})

			# x:126 y:316
			OperatableStateMachine.add('Go_To_Door_Ready_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_READY_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Go_To_Open_Door_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'side': 'hand_side'})

			# x:100 y:203
			OperatableStateMachine.add('Go_To_Stand',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.5, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Go_To_Door_Ready_Pose', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'side': 'hand_side'})

			# x:551 y:580
			OperatableStateMachine.add('Back_To_Door_Ready_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_READY_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Go_To_Final_Stand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'side': 'hand_side'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
