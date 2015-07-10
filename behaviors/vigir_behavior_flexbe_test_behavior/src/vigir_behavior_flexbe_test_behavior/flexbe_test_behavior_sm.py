#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_flexbe_test_behavior')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.wait_state import WaitState
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.calculation_state import CalculationState
from vigir_behavior_flexbe_inner_test_behavior.flexbe_inner_test_behavior_sm import FlexBEInnerTestBehaviorSM
from flexbe_states.input_state import InputState
from flexbe_states.concurrent_state import ConcurrentState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
#from vigir_behavior_flexbe_testprint_behavior.flexbe_testprint_behavior_sm import FlexBETestprintBehaviorSM
from vigir_behavior_flexbe_inner_test_behavior.flexbe_inner_test_behavior_sm import FlexBEInnerTestBehaviorSM
import rospy
# [/MANUAL_IMPORT]


'''
Created on Tue Nov 18 2014
@author: Philipp Schillinger
'''
class FlexBETestBehaviorSM(Behavior):
	'''
	A simple test behavior for testing FlexBE
	'''


	def __init__(self):
		super(FlexBETestBehaviorSM, self).__init__()
		self.name = 'FlexBE Test Behavior'

		# parameters of this behavior
		self.add_parameter('wait_time', 3)

		# references to used behaviors
		self.add_behavior(FlexBEInnerTestBehaviorSM, 'Embedded_Statemachine/FlexBE Inner Test Behavior')
		self.add_behavior(FlexBEInnerTestBehaviorSM, 'Embedded_Statemachine/FlexBE Inner Test Behavior 2')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		concurrent_states = dict()
		concurrent_mapping = list()
		welcome_log = "Performing Tests..."
		# x:219 y:439, x:562 y:308
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.input_value = "bla"
		_state_machine.userdata.blubb = "lalala"
		_state_machine.userdata.embedded_input = 4
		_state_machine.userdata.behavior_my_input = 8

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		concurrent_states['wait_short'] = WaitState(wait_time=2)
		concurrent_states['wait_long'] = WaitState(wait_time=4)
		#concurrent_states['calc_one'] = CalculationState(lambda x: "bla1")
		#concurrent_states['calc_two'] = FlexibleCalculationState(lambda x: "bla2", ["input1", "input2"])
		concurrent_states['calc_print'] = CalculationState(self.print_input)
		#concurrent_states['behavior'] = FlexBEInnerTestBehaviorSM()
		#concurrent_states['behavior'].message = "Hello World!"
		concurrent_mapping.append({'outcome': 'done', 'condition': {'wait_short': 'done', 'wait_long': 'done'}})


		# [/MANUAL_CREATE]

		# x:88 y:457, x:340 y:145
		_sm_embedded_statemachine_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['inner_value', 'embedded_input'])

		with _sm_embedded_statemachine_0:
			# x:30 y:40
			OperatableStateMachine.add('Inner_Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'Decision'},
										autonomy={'done': Autonomy.Low})

			# x:37 y:165
			OperatableStateMachine.add('Decision',
										OperatorDecisionState(outcomes=["finished", "failed"], hint="Choose one outcome.", suggestion="finished"),
										transitions={'finished': 'FlexBE Inner Test Behavior', 'failed': 'failed'},
										autonomy={'finished': Autonomy.High, 'failed': Autonomy.Low})

			# x:41 y:308
			OperatableStateMachine.add('Second_Inner_wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low})

			# x:317 y:323
			OperatableStateMachine.add('Inner_Calc',
										CalculationState(calculation=self.inner_calc),
										transitions={'done': 'Second_Inner_wait'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'inner_value', 'output_value': 'output_value'})

			# x:258 y:229
			OperatableStateMachine.add('FlexBE Inner Test Behavior',
										self.use_behavior(FlexBEInnerTestBehaviorSM, 'Embedded_Statemachine/FlexBE Inner Test Behavior'),
										transitions={'finished': 'FlexBE Inner Test Behavior 2', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'my_input': 'embedded_input'})

			# x:502 y:217
			OperatableStateMachine.add('FlexBE Inner Test Behavior 2',
										self.use_behavior(FlexBEInnerTestBehaviorSM, 'Embedded_Statemachine/FlexBE Inner Test Behavior 2'),
										transitions={'finished': 'Inner_Calc', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'my_input': 'embedded_input'})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Initial_Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'TestInput'},
										autonomy={'done': Autonomy.High})

			# x:202 y:40
			OperatableStateMachine.add('Welcome_Log',
										LogState(text=welcome_log, severity=0),
										transitions={'done': 'Second_Wait'},
										autonomy={'done': Autonomy.Low})

			# x:646 y:39
			OperatableStateMachine.add('Second_Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'Any_Calculation'},
										autonomy={'done': Autonomy.Low})

			# x:364 y:184
			OperatableStateMachine.add('Embedded_Statemachine',
										_sm_embedded_statemachine_0,
										transitions={'finished': 'Final_Wait', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'inner_value': 'output_value', 'embedded_input': 'embedded_input'})

			# x:201 y:188
			OperatableStateMachine.add('Final_Wait',
										WaitState(wait_time=self.wait_time),
										transitions={'done': 'Decide_Back'},
										autonomy={'done': Autonomy.High})

			# x:183 y:316
			OperatableStateMachine.add('Decide_Back',
										OperatorDecisionState(outcomes=["back", "done"], hint="Repeat inner state machine?", suggestion="back"),
										transitions={'back': 'Log_Go_Back', 'done': 'finished'},
										autonomy={'back': Autonomy.High, 'done': Autonomy.Low})

			# x:639 y:190
			OperatableStateMachine.add('Any_Calculation',
										CalculationState(calculation=self.any_calculation),
										transitions={'done': 'Embedded_Statemachine'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input_value', 'output_value': 'output_value'})

			# x:33 y:147
			OperatableStateMachine.add('TestInput',
										InputState(request=0, message="I need some data!"),
										transitions={'received': 'Welcome_Log', 'aborted': 'Test_Concurrency', 'no_connection': 'Final_Wait', 'data_error': 'Decide_Back'},
										autonomy={'received': Autonomy.Full, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'output_value'})

			# x:288 y:100 {?input_keys = ['calc_print_input_value', 'behavior_my_input'],?output_keys = ['calc_print_output_value']}
			OperatableStateMachine.add('Test_Concurrency',
										ConcurrentState(states=concurrent_states, outcomes=['done'], outcome_mapping=concurrent_mapping),
										transitions={'done': 'Test_Output'},
										autonomy={'done': Autonomy.Off},
										remapping={'calc_print_input_value': 'input_value', 'behavior_my_input': 'behavior_my_input', 'calc_print_output_value': 'concurent_output'})

			# x:491 y:99
			OperatableStateMachine.add('Test_Output',
										CalculationState(calculation=self.print_input),
										transitions={'done': 'Second_Wait'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'concurent_output', 'output_value': 'output_value'})

			# x:397 y:316
			OperatableStateMachine.add('Log_Go_Back',
										LogState(text="Going back!", severity=Logger.REPORT_INFO),
										transitions={'done': 'Embedded_Statemachine'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def any_calculation(self, input_value):
		Logger.loginfo("Any Calc: %s" % str(self._state_machine.userdata._data))
		Logger.loginfo("Own input: %s" % str(input_value))
		return "Blubb"

	def inner_calc(self, inner_value):
		Logger.loginfo("Inner Calc: %s " % str(self._state_machine.userdata._data))
		Logger.loginfo("Inner Calc: %s " % str(list(self._state_machine.userdata._data.keys())))
		Logger.loginfo("Own input: %s" % str(inner_value))
		return "Bl"

	def print_input(self, input_value):
		rospy.loginfo(str(input_value))
		return "printed"
	# [/MANUAL_FUNC]
