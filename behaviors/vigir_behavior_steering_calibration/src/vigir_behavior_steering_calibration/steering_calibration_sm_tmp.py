#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_steering_calibration')
from vigir_be_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_be_states.log_state import LogState
from vigir_be_states.input_state import InputState
from vigir_be_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Aug 14 2015
@author: Achim Stein
'''
class SteeringCalibrationSM(Behavior):
	'''
	Calibrates the steering motion necessary for the first DRC finals task (driving)
	'''


	def __init__(self):
		super(SteeringCalibrationSM, self).__init__()
		self.name = 'Steering Calibration'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:77 y:539, x:230 y:506
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:412 y:66, x:87 y:494
		_sm_statemachine_0 = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['wall_template_id'])

		with _sm_statemachine_0:
			# x:71 y:85
			OperatableStateMachine.add('Request_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the STEERING_WHEEL template."),
										transitions={'received': 'finished', 'aborted': 'failed', 'no_connection': 'Log_No_Connection', 'data_error': 'Log_Data_Error'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:167 y:234
			OperatableStateMachine.add('Log_Data_Error',
										LogState(text="Received wrong data format!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})

			# x:328 y:236
			OperatableStateMachine.add('Log_No_Connection',
										LogState(text='No Connection', severity=Logger.REPORT_HINT),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})



		with _state_machine:
			# x:94 y:104
			OperatableStateMachine.add('Notify_Execution_Started',
										LogState(text='Execution has started. Consider making modifications if this is a re-run.', severity=Logger.REPORT_HINT),
										transitions={'done': 'Statemachine'},
										autonomy={'done': Autonomy.Full})

			# x:30 y:174
			OperatableStateMachine.add('Statemachine',
										_sm_statemachine_0,
										transitions={'finished': 'Set_Manipulate', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'wall_template_id': 'wall_template_id'})

			# x:321 y:246
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Notify_Behavior_Exit', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:179 y:613
			OperatableStateMachine.add('Notify_Behavior_Exit',
										LogState(text='The behavior will now exit. Last chance to intervene!', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def _calc_target_angle():
		target_angle = target_angle + angle_increment
		return target_angle
	
	def _check_continue_rotation(target_angle):		
		if ( target_angle >= 360.0 ):
			return 'finished'
		else:			
			return 'continue_rotation'
		
	# [/MANUAL_FUNC]
