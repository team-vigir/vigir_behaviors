#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_flexbe_inner_test_behavior')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from flexbe_states.wait_state import WaitState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.calculation_state import CalculationState
from vigir_behavior_flexbe_testprint_behavior.flexbe_testprint_behavior_sm import FlexBETestprintBehaviorSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Feb 26 2015
@author: Philipp Schillinger
'''
class FlexBEInnerTestBehaviorSM(Behavior):
	'''
	A behavior to be embedded into another behvaior for testing.
	'''


	def __init__(self):
		super(FlexBEInnerTestBehaviorSM, self).__init__()
		self.name = 'FlexBE Inner Test Behavior'

		# parameters of this behavior
		self.add_parameter('message', 'Hello!')

		# references to used behaviors
		self.add_behavior(FlexBETestprintBehaviorSM, 'FlexBE Testprint Behavior')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]


	def create(self):
		# x:119 y:369, x:527 y:258
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['my_input'])
		_state_machine.userdata.my_input = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Entry_Msg',
										LogState(text="Now at inner behavior...", severity=Logger.REPORT_INFO),
										transitions={'done': 'FlexBE Testprint Behavior'},
										autonomy={'done': Autonomy.Off})

			# x:34 y:240
			OperatableStateMachine.add('Wait_A_Bit',
										WaitState(wait_time=2),
										transitions={'done': 'Print_State'},
										autonomy={'done': Autonomy.High})

			# x:279 y:327
			OperatableStateMachine.add('Decide_Outcome',
										OperatorDecisionState(outcomes=['finished', 'failed'], hint="How should this end?", suggestion='finished'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Full, 'failed': Autonomy.Full})

			# x:289 y:127
			OperatableStateMachine.add('Print_State',
										CalculationState(calculation=self.print_number_and_inc),
										transitions={'done': 'Print_Param_Message'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'my_input', 'output_value': 'output_value'})

			# x:273 y:212
			OperatableStateMachine.add('Print_Param_Message',
										LogState(text="Wait, there is a message: " + self.message, severity=Logger.REPORT_HINT),
										transitions={'done': 'Decide_Outcome'},
										autonomy={'done': Autonomy.Off})

			# x:15 y:124
			OperatableStateMachine.add('FlexBE Testprint Behavior',
										self.use_behavior(FlexBETestprintBehaviorSM, 'FlexBE Testprint Behavior'),
										transitions={'finished': 'Wait_A_Bit'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	def print_number_and_inc(self, input_value):
		Logger.loginfo("Got value: %s" % str(input_value))
		return input_value + 1

	# [/MANUAL_FUNC]
