#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_stand_up_behavior')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_flexbe_states.motion_service_state import MotionServiceState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon May 11 2015
@author: Dorothea Koert
'''
class StandUpBehaviorSM(Behavior):
	'''
	
	'''


	def __init__(self):
		super(StandUpBehaviorSM, self).__init__()
		self.name = 'Stand Up Behavior'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		motion_key_1 = stand_up_front_100
		motion_key_2 = stand_up_front_200
		# x:533 y:194, x:130 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Execute_First_Part',
										MotionServiceState(motion_key=motion_key_1, time_factor=1),
										transitions={'done': 'Execute_Second_Part', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Low})

			# x:203 y:125
			OperatableStateMachine.add('Execute_Second_Part',
										MotionServiceState(motion_key=motion_key_2, time_factor=1),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Low})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
