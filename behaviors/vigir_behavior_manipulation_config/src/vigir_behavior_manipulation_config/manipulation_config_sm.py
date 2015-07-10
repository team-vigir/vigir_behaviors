#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_manipulation_config')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from flexbe_atlas_states.footstep_plan_wide_stance_state import FootstepPlanWideStanceState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_atlas_states.check_current_control_mode_state import CheckCurrentControlModeState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sun Apr 19 2015
@author: Philipp Schillinger
'''
class ManipulationConfigSM(Behavior):
	'''
	Prepares the robot for manipulation.
	'''


	def __init__(self):
		super(ManipulationConfigSM, self).__init__()
		self.name = 'Manipulation Config'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:833 y:90, x:583 y:490
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:37 y:78
			OperatableStateMachine.add('Ask_If_Wide_Stance',
										OperatorDecisionState(outcomes=['wide_stance', 'just_stand'], hint="Do you want to go to wide stance?", suggestion='wide_stance'),
										transitions={'wide_stance': 'Plan_To_Wide_Stance', 'just_stand': 'Set_Manipulate'},
										autonomy={'wide_stance': Autonomy.Low, 'just_stand': Autonomy.Full})

			# x:274 y:278
			OperatableStateMachine.add('Go_To_Wide_Stance',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Wait_For_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'footstep_plan': 'plan_wide_stance'})

			# x:19 y:278
			OperatableStateMachine.add('Plan_To_Wide_Stance',
										FootstepPlanWideStanceState(),
										transitions={'planned': 'Go_To_Wide_Stance', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'footstep_plan': 'plan_wide_stance'})

			# x:566 y:78
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})

			# x:266 y:178
			OperatableStateMachine.add('Wait_For_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=True),
										transitions={'correct': 'Set_Manipulate', 'incorrect': 'failed'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Full},
										remapping={'control_mode': 'control_mode'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
