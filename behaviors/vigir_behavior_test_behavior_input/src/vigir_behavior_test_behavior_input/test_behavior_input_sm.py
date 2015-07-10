#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_test_behavior_input')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from flexbe_states.log_state import LogState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.concurrent_state import ConcurrentState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import os
import time
import glob

import rospy
from control_msgs.msg import *
from trajectory_msgs.msg import *
# [/MANUAL_IMPORT]


'''
Created on Wed Apr 01 2015
@author: Philipp
'''
class TestBehaviorInputSM(Behavior):
	'''
	Simple behavior for testing the Behavior Input pipeline.
	'''


	def __init__(self):
		super(TestBehaviorInputSM, self).__init__()
		self.name = 'Test Behavior Input'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		

		# [/MANUAL_INIT]


	def create(self):
		POINT_LOCATION = 0
		SELECTED_OBJECT_ID = 1
		WAYPOINT_GOAL_POSE = 2
		GHOST_JOINT_STATES = 3
		concurrent_states = dict()
		concurrent_mapping = list()
		type1 = POINT_LOCATION
		type2 = SELECTED_OBJECT_ID
		# x:717 y:559, x:344 y:460
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		concurrent_states['input1'] = InputState(request=type1, message="Please give me data 1...")
		concurrent_states['input2'] = InputState(request=type2, message="Please give me data 2...")
		concurrent_states['calc'] = CalculationState(lambda x: 0)
		concurrent_mapping.append({'outcome': 'done', 'condition': {'input1': 'received', 'input2': 'received'}})

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Request_Data',
										InputState(request=SELECTED_OBJECT_ID, message="Please give me some data..."),
										transitions={'received': 'Log_Received', 'aborted': 'Log_Aborted', 'no_connection': 'Log_No_Connection', 'data_error': 'Log_Data_Error'},
										autonomy={'received': Autonomy.High, 'aborted': Autonomy.High, 'no_connection': Autonomy.High, 'data_error': Autonomy.High},
										remapping={'data': 'data'})

			# x:30 y:247
			OperatableStateMachine.add('Log_Received',
										LogState(text="Got behavior data!", severity=Logger.REPORT_INFO),
										transitions={'done': 'Print_Result'},
										autonomy={'done': Autonomy.Off})

			# x:324 y:255
			OperatableStateMachine.add('Log_Aborted',
										LogState(text="Operator aborted data request.", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:489 y:241
			OperatableStateMachine.add('Log_No_Connection',
										LogState(text="Unable to connect to OCS!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:158 y:243
			OperatableStateMachine.add('Log_Data_Error',
										LogState(text="Failed to deserialize data.", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:27 y:411
			OperatableStateMachine.add('Print_Result',
										CalculationState(calculation=self.print_input_data),
										transitions={'done': 'Concurrent_Request'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'data', 'output_value': 'output_value'})

			# x:242 y:586 {?input_keys = ['calc_input_value'],?output_keys = ['input1_data', 'input2_data', 'calc_output_value']}
			OperatableStateMachine.add('Concurrent_Request',
										ConcurrentState(states=concurrent_states, outcomes=['done', 'failed'], outcome_mapping=concurrent_mapping),
										transitions={'done': 'Print_Result_1', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'calc_input_value': 'data', 'input1_data': 'input1_data', 'input2_data': 'input2_data', 'calc_output_value': 'calc_output_value'})

			# x:520 y:481
			OperatableStateMachine.add('Print_Result_1',
										CalculationState(calculation=self.print_input_data),
										transitions={'done': 'Print_Result_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input1_data', 'output_value': 'output_value'})

			# x:495 y:646
			OperatableStateMachine.add('Print_Result_2',
										CalculationState(calculation=self.print_input_data),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'input2_data', 'output_value': 'output_value'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	def print_input_data(self, data):
		print "This is my data:"
		print data
		return data

	# [/MANUAL_FUNC]
