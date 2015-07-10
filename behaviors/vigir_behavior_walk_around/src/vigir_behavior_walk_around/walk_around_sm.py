#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_walk_around')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.footstep_plan_relative_state import FootstepPlanRelativeState
from flexbe_atlas_states.footstep_plan_turn_state import FootstepPlanTurnState
from flexbe_atlas_states.footstep_plan_wide_stance_state import FootstepPlanWideStanceState
from flexbe_atlas_states.footstep_plan_realign_center_state import FootstepPlanRealignCenterState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Feb 25 2015
@author: Philipp
'''
class WalkAroundSM(Behavior):
	'''
	Test walking states.
	'''


	def __init__(self):
		super(WalkAroundSM, self).__init__()
		self.name = 'Walk Around'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:111 y:265, x:563 y:696
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.distance = 2 # m
		_state_machine.userdata.angle = 60 # deg

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:63 y:118
			OperatableStateMachine.add('Where_Should_I_Go?',
										OperatorDecisionState(outcomes=['walk', 'turn', 'special', 'done'], hint=None, suggestion=None),
										transitions={'walk': 'Decide_Walking_Direction', 'turn': 'Decide_Turning_Direction', 'special': 'Decide_Special_Footstep_Plan', 'done': 'finished'},
										autonomy={'walk': Autonomy.Off, 'turn': Autonomy.Off, 'special': Autonomy.Off, 'done': Autonomy.Off})

			# x:280 y:112
			OperatableStateMachine.add('Decide_Walking_Direction',
										OperatorDecisionState(outcomes=['forward', 'backward', 'left', 'right'], hint=None, suggestion=None),
										transitions={'forward': 'Walk_Forward', 'backward': 'Walk_Backward', 'left': 'Walk_Left', 'right': 'Walk_Right'},
										autonomy={'forward': Autonomy.Off, 'backward': Autonomy.Off, 'left': Autonomy.Off, 'right': Autonomy.Off})

			# x:288 y:321
			OperatableStateMachine.add('Decide_Turning_Direction',
										OperatorDecisionState(outcomes=['left', 'right'], hint=None, suggestion=None),
										transitions={'left': 'Turn_Left', 'right': 'Turn_Right'},
										autonomy={'left': Autonomy.Off, 'right': Autonomy.Off})

			# x:271 y:503
			OperatableStateMachine.add('Decide_Special_Footstep_Plan',
										OperatorDecisionState(outcomes=['widestance', 'realign'], hint=None, suggestion=None),
										transitions={'widestance': 'Wide_Stance', 'realign': 'Realign'},
										autonomy={'widestance': Autonomy.Off, 'realign': Autonomy.Off})

			# x:506 y:18
			OperatableStateMachine.add('Walk_Forward',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_FORWARD),
										transitions={'planned': 'Execute_Plan', 'failed': 'Turn_Right'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'footstep_plan': 'footstep_plan'})

			# x:506 y:85
			OperatableStateMachine.add('Walk_Backward',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_BACKWARD),
										transitions={'planned': 'Execute_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'footstep_plan': 'footstep_plan'})

			# x:506 y:151
			OperatableStateMachine.add('Walk_Left',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_LEFT),
										transitions={'planned': 'Execute_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'footstep_plan': 'footstep_plan'})

			# x:507 y:215
			OperatableStateMachine.add('Walk_Right',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_RIGHT),
										transitions={'planned': 'Execute_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'distance': 'distance', 'footstep_plan': 'footstep_plan'})

			# x:511 y:297
			OperatableStateMachine.add('Turn_Left',
										FootstepPlanTurnState(direction=FootstepPlanTurnState.TURN_LEFT),
										transitions={'planned': 'Execute_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'angle': 'angle', 'footstep_plan': 'footstep_plan'})

			# x:511 y:365
			OperatableStateMachine.add('Turn_Right',
										FootstepPlanTurnState(direction=FootstepPlanTurnState.TURN_RIGHT),
										transitions={'planned': 'Execute_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'angle': 'angle', 'footstep_plan': 'footstep_plan'})

			# x:493 y:464
			OperatableStateMachine.add('Wide_Stance',
										FootstepPlanWideStanceState(),
										transitions={'planned': 'Execute_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'footstep_plan': 'footstep_plan'})

			# x:492 y:532
			OperatableStateMachine.add('Realign',
										FootstepPlanRealignCenterState(),
										transitions={'planned': 'Execute_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'footstep_plan': 'footstep_plan'})

			# x:856 y:257
			OperatableStateMachine.add('Execute_Plan',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Where_Should_I_Go?', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'footstep_plan': 'footstep_plan'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
