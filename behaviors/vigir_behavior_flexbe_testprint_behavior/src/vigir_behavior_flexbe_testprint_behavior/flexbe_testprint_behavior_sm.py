#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_flexbe_testprint_behavior')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Mar 02 2015
@author: Philipp
'''
class FlexBETestprintBehaviorSM(Behavior):
	'''
	Just prints its parameter
	'''


	def __init__(self):
		super(FlexBETestprintBehaviorSM, self).__init__()
		self.name = 'FlexBE Testprint Behavior'

		# parameters of this behavior
		self.add_parameter('msg', 'test')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]


	def create(self):
		# x:30 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:66 y:154
			OperatableStateMachine.add('Print_Msg',
										LogState(text="My param is: " + self.msg, severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
