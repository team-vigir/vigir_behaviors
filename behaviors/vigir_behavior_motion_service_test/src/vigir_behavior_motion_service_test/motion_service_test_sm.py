#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_motion_service_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_atlas_states.motion_service_state import MotionServiceState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 06 2015
@author: Martin Oehler
'''
class MotionServiceTestSM(Behavior):
	'''
	Test for executing motions via actions
	'''


	def __init__(self):
		super(MotionServiceTestSM, self).__init__()
		self.name = 'Motion Service Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:521 y:567, x:495 y:147
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:47 y:147
			OperatableStateMachine.add('ready state',
										MotionServiceState(motion_key='ready_state', time_factor=1),
										transitions={'done': 'greet left', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:48 y:240
			OperatableStateMachine.add('greet left',
										MotionServiceState(motion_key='cebit_greet_left', time_factor=1),
										transitions={'done': 'greet right', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:50 y:331
			OperatableStateMachine.add('greet right',
										MotionServiceState(motion_key='cebit_greet_right', time_factor=1),
										transitions={'done': 'muscle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:52 y:420
			OperatableStateMachine.add('muscle',
										MotionServiceState(motion_key='cebit_muscle', time_factor=1),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
