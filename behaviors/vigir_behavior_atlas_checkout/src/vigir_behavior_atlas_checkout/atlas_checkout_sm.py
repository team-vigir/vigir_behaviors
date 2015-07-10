#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_atlas_checkout')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.calculation_state import CalculationState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.log_state import LogState
from flexbe_states.decision_state import DecisionState
from flexbe_atlas_states.robot_state_command_state import RobotStateCommandState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_behavior_praying_mantis_calibration.praying_mantis_calibration_sm import PrayingMantisCalibrationSM
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_atlas_states.tilt_head_state import TiltHeadState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

from flor_control_msgs.msg import FlorRobotStatus
from flexbe_core.proxy import ProxySubscriberCached

# [/MANUAL_IMPORT]


'''
Created on Sat May 23 2015
@author: Spyros Maniatopoulos
'''
class ATLAScheckoutSM(Behavior):
	'''
	A behavior meant to be run before high voltage is provided to ATLAS. It guides the robot through BDI's calibration, initial control mode changes, Praying Mantis calibration, and visual calibration check.
	'''


	def __init__(self):
		super(ATLAScheckoutSM, self).__init__()
		self.name = 'ATLAS checkout'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(PrayingMantisCalibrationSM, 'Praying Mantis Calibration')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		self._status_topic = '/flor/controller/robot_status'
		self._sub =  ProxySubscriberCached({self._status_topic: FlorRobotStatus})
		self._sub.make_persistant(self._status_topic)
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1125 y:169, x:430 y:273
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.left = 'left'
		_state_machine.userdata.right = 'right'
		_state_machine.userdata.none = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:728 y:81, x:318 y:310
		_sm_confirm_calibration_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['left', 'right'])

		with _sm_confirm_calibration_0:
			# x:68 y:71
			OperatableStateMachine.add('Go_to_MANIPULATE',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Look_Slightly_Down', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})

			# x:75 y:301
			OperatableStateMachine.add('Check_Left_Arm',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CALIBRATE_ARMS, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Bring_Left_Arm_Down', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Low},
										remapping={'side': 'left'})

			# x:76 y:394
			OperatableStateMachine.add('Bring_Left_Arm_Down',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.SINGLE_ARM_STAND, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Check_Right_Arm', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'side': 'left'})

			# x:426 y:393
			OperatableStateMachine.add('Check_Right_Arm',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.CALIBRATE_ARMS, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Bring_Right_Arm_Down', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Low},
										remapping={'side': 'right'})

			# x:426 y:301
			OperatableStateMachine.add('Bring_Right_Arm_Down',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.SINGLE_ARM_STAND, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Look_Straight', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'side': 'right'})

			# x:415 y:75
			OperatableStateMachine.add('Go_to_STAND',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Low})

			# x:81 y:185
			OperatableStateMachine.add('Look_Slightly_Down',
										TiltHeadState(desired_tilt=TiltHeadState.DOWN_30),
										transitions={'done': 'Check_Left_Arm', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low})

			# x:449 y:178
			OperatableStateMachine.add('Look_Straight',
										TiltHeadState(desired_tilt=TiltHeadState.STRAIGHT),
										transitions={'done': 'Go_to_STAND', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low})


		# x:748 y:57, x:306 y:173
		_sm_checks_and_bdi_calibration_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none'])

		with _sm_checks_and_bdi_calibration_1:
			# x:54 y:28
			OperatableStateMachine.add('Get_Air_Pressure',
										CalculationState(calculation=self.get_air_sump_pressure),
										transitions={'done': 'Check_Air_Pressure'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'air_pressure'})

			# x:46 y:411
			OperatableStateMachine.add('Check_Hands_Initialized',
										OperatorDecisionState(outcomes=['ready', 'again'], hint='Did both hands initialize?', suggestion='ready'),
										transitions={'ready': 'Turn_Pump_ON', 'again': 'Request_Robot_Power'},
										autonomy={'ready': Autonomy.High, 'again': Autonomy.Full})

			# x:267 y:29
			OperatableStateMachine.add('Air_Pressure_Warning',
										LogState(text='Check the air pressure! Has to be 120 +/- 10 PSI.', severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Full})

			# x:50 y:149
			OperatableStateMachine.add('Check_Air_Pressure',
										DecisionState(outcomes=['ok', 'alert'], conditions=lambda p: 'ok' if p > 110.0 and p <= 130.0 else 'alert'),
										transitions={'ok': 'Request_Robot_Power', 'alert': 'Air_Pressure_Warning'},
										autonomy={'ok': Autonomy.Full, 'alert': Autonomy.Low},
										remapping={'input_value': 'air_pressure'})

			# x:500 y:286
			OperatableStateMachine.add('Calibrate_ARMS',
										RobotStateCommandState(command=RobotStateCommandState.CALIBRATE_ARMS),
										transitions={'done': 'FREEZE_in_Between', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Low})

			# x:499 y:52
			OperatableStateMachine.add('Calibrate_BIASES',
										RobotStateCommandState(command=RobotStateCommandState.CALIBRATE),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.Low})

			# x:488 y:167
			OperatableStateMachine.add('FREEZE_in_Between',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.FREEZE),
										transitions={'changed': 'Calibrate_BIASES', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:500 y:412
			OperatableStateMachine.add('Turn_Pump_ON',
										RobotStateCommandState(command=RobotStateCommandState.START_HYDRAULIC_PRESSURE_ON),
										transitions={'done': 'Calibrate_ARMS', 'failed': 'failed'},
										autonomy={'done': Autonomy.Full, 'failed': Autonomy.High})

			# x:47 y:276
			OperatableStateMachine.add('Request_Robot_Power',
										LogState(text='Request robot power from field team.', severity=Logger.REPORT_HINT),
										transitions={'done': 'Check_Hands_Initialized'},
										autonomy={'done': Autonomy.Full})



		with _state_machine:
			# x:74 y:88
			OperatableStateMachine.add('Checks_and_BDI_Calibration',
										_sm_checks_and_bdi_calibration_1,
										transitions={'finished': 'Go_to_STAND_PREP', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none'})

			# x:79 y:264
			OperatableStateMachine.add('Go_to_STAND_PREP',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND_PREP),
										transitions={'changed': 'Go_to_STAND', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Full, 'failed': Autonomy.Low})

			# x:78 y:393
			OperatableStateMachine.add('Go_to_STAND',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Praying Mantis Calibration', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Full, 'failed': Autonomy.Low})

			# x:611 y:388
			OperatableStateMachine.add('Praying Mantis Calibration',
										self.use_behavior(PrayingMantisCalibrationSM, 'Praying Mantis Calibration'),
										transitions={'finished': 'Decide_Calibration_Check', 'failed': 'Decide_Retry'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:624 y:249
			OperatableStateMachine.add('Decide_Calibration_Check',
										OperatorDecisionState(outcomes=['check', 'skip'], hint='Do you want to confirm the arm calibration visually?', suggestion='skip'),
										transitions={'check': 'Confirm_Calibration', 'skip': 'Logging_Reminder'},
										autonomy={'check': Autonomy.Full, 'skip': Autonomy.High})

			# x:627 y:93
			OperatableStateMachine.add('Confirm_Calibration',
										_sm_confirm_calibration_0,
										transitions={'finished': 'Logging_Reminder', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'left': 'left', 'right': 'right'})

			# x:389 y:340
			OperatableStateMachine.add('Decide_Retry',
										OperatorDecisionState(outcomes=['retry', 'fail'], hint='Try running Praying Mantis Calibration again?', suggestion='retry'),
										transitions={'retry': 'Praying Mantis Calibration', 'fail': 'failed'},
										autonomy={'retry': Autonomy.High, 'fail': Autonomy.Full})

			# x:907 y:164
			OperatableStateMachine.add('Logging_Reminder',
										LogState(text='Start video logging', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	def get_air_sump_pressure(self, input):
		'''Returns the current air sump pressure.'''
		
		robot_status = self._sub.get_last_msg(self._status_topic)

		air_pressure = robot_status.air_sump_pressure
		Logger.loginfo('Air Sump Pressure: %.2f' % air_pressure)

		return air_pressure

	# [/MANUAL_FUNC]
