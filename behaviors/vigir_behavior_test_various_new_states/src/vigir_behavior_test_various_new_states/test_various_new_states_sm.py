#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_test_various_new_states')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jan 30 2015
@author: Spyros Maniatopoulos
'''
class TestVariousNewStatesSM(Behavior):
	'''
	This is a dummy behavior. It is used for debugging new states that are currently under development.
	'''


	def __init__(self):
		super(TestVariousNewStatesSM, self).__init__()
		self.name = 'Test Various New States'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:185 y:376, x:483 y:51
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:113 y:43
			OperatableStateMachine.add('Change_to_MANIPULATE',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.High, 'failed': Autonomy.Low})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
