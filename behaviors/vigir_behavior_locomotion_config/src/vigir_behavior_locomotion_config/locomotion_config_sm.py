#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_locomotion_config')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_flexbe_states.check_current_control_mode_state import CheckCurrentControlModeState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_flexbe_states.footstep_plan_realign_center_state import FootstepPlanRealignCenterState
from vigir_flexbe_states.execute_step_plan_action_state import ExecuteStepPlanActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Apr 19 2015
@author: Philipp Schillinger
'''
class LocomotionConfigSM(Behavior):
	'''
	Prepares the robot for walking after being in manipulation config.
	'''


	def __init__(self):
		super(LocomotionConfigSM, self).__init__()
		self.name = 'Locomotion Config'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:533 y:590, x:583 y:140
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.none = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:78
			OperatableStateMachine.add('Check_Manipulate',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.MANIPULATE, wait=False),
										transitions={'correct': 'Set_Stand_Posture', 'incorrect': 'Set_Manipulate'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Low},
										remapping={'control_mode': 'control_mode'})

			# x:266 y:128
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Set_Stand_Posture', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})

			# x:76 y:228
			OperatableStateMachine.add('Set_Stand_Posture',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Set_Stand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'side': 'none'})

			# x:333 y:378
			OperatableStateMachine.add('Realign_Feet_Decision',
										OperatorDecisionState(outcomes=["realign", "just_stand"], hint="Need to realign the feet after wide stance?", suggestion="realign"),
										transitions={'realign': 'Plan_Realign', 'just_stand': 'finished'},
										autonomy={'realign': Autonomy.Low, 'just_stand': Autonomy.Full})

			# x:464 y:278
			OperatableStateMachine.add('Plan_Realign',
										FootstepPlanRealignCenterState(),
										transitions={'planned': 'Execute_Realign', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'plan_header': 'plan_header'})

			# x:524 y:378
			OperatableStateMachine.add('Execute_Realign',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Wait_For_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'plan_header': 'plan_header'})

			# x:66 y:328
			OperatableStateMachine.add('Set_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Realign_Feet_Decision', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})

			# x:766 y:378
			OperatableStateMachine.add('Wait_For_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=False),
										transitions={'correct': 'finished', 'incorrect': 'failed'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Full},
										remapping={'control_mode': 'control_mode'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
