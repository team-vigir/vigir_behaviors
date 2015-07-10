#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_sinusoide_joint_control_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_atlas_states.check_current_control_mode_state import CheckCurrentControlModeState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_atlas_states.moveit_move_group_state import MoveitMoveGroupState
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from flexbe_atlas_states.read_dynamic_parameter_state import ReadDynamicParameterState
from flexbe_states.start_record_logs_state import StartRecordLogsState
from flexbe_states.stop_record_logs_state import StopRecordLogsState
from flexbe_atlas_states.update_dynamic_parameter_state import UpdateDynamicParameterState
from flexbe_states.wait_state import WaitState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import time
import os
import rospy
from flexbe_core import EventState
# [/MANUAL_IMPORT]


'''
Created on Mon Nov 03 2014
@author: Philipp and Spyros
'''
class SinusoideJointControlTestSM(Behavior):
	'''
	Create sinusoide motions for all joints.
	'''


	def __init__(self):
		super(SinusoideJointControlTestSM, self).__init__()
		self.name = 'Sinusoide Joint Control Test'

		# parameters of this behavior
		self.add_parameter('topics_to_record', '')
		self.add_parameter('real_robot', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

# 0-5	left arm
# 6-11	right arm
# for each: wrx, wry, elx, ely, shx, shz

# simulation
		self._joint_limits_sim = [ \
			[-0.44, 1.57], \
			[-1.57, 1.57], \
			[0.00, 2.35], \
			[0.00, 3.14], \
			[-1.40, 1.75], \
			[-1.96, 1.96], \
			 \
			[-1.57, 0.44], \
			[-1.57, 1.57], \
			[-2.35, 0.00], \
			[0.00, 3.14], \
			[-1.75, 1.40], \
			[-1.96, 1.96] \
		]
# real robot
		self._joint_limits_rob = [ \
			[-1.18, 1.18], \
			[0.00, 3.14], \
			[0.00, 2.36], \
			[0.00, 3.14], \
			[-1.57, 1.57], \
			[-1.57, 0.79], \
			 \
			[-1.18, 1.18], \
			[0.00, 3.14], \
			[-2.36, 0.00], \
			[0.00, 3.14], \
			[-1.57, 1.57], \
			[-1.57, 0.79], \
		]

		self._joint_limits = []

		# joint order: shz, shx, ely, elx, wry, wrx
		self._joint_configs_down = []
		self._joint_configs_up = []

		self._traj_controllers = [ \
			UpdateDynamicParameterState.LEFT_ARM_WRX, \
			UpdateDynamicParameterState.LEFT_ARM_WRY, \
			UpdateDynamicParameterState.LEFT_ARM_ELX, \
			UpdateDynamicParameterState.LEFT_ARM_ELY, \
			UpdateDynamicParameterState.LEFT_ARM_SHX, \
			UpdateDynamicParameterState.LEFT_ARM_SHZ, \
			\
			UpdateDynamicParameterState.RIGHT_ARM_WRX, \
			UpdateDynamicParameterState.RIGHT_ARM_WRY, \
			UpdateDynamicParameterState.RIGHT_ARM_ELX, \
			UpdateDynamicParameterState.RIGHT_ARM_ELY, \
			UpdateDynamicParameterState.RIGHT_ARM_SHX, \
			UpdateDynamicParameterState.RIGHT_ARM_SHZ \
		]

		self._amplitudes = [] # given as private variable
		self._frequencies = [] # given as private variable
		
		# [/MANUAL_INIT]


	def create(self):
		joint_names_left = ["l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx"]
		joint_names_right = ["r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx"]
		wait_time = 10.0
		bagfolder = "" # calculated
		gains_list = {'pid_gains': ['p', 'i', 'd'], 'bdi_gains': ['k_qd_p', 'ff_qd_d'], 'vigir_gains': ['ff_bang', 'ff_effort', 'ff_friction']}
		amplitudes = [2, 5, 10] # set i to 1
		frequencies = [1.26, 3.14, 6.28]
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joints_left_up = [] # calculated
		_state_machine.userdata.joints_right_up = [] # calculated
		_state_machine.userdata.joint_index = 0
		_state_machine.userdata.none = None		

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# 'Basic' configuration for SIMULATION
		#_state_machine.userdata.joints_left_up = [0.00, 0.18, 1.57, 1.18, 0.00, 0.57]
		#_state_machine.userdata.joints_right_up = [0.00, -0.18, 1.57, -1.18, 0.00, -0.57]

		logs_folder = os.path.expanduser('~/joint_control_tests/')
		if not os.path.exists(logs_folder):
    			os.makedirs(logs_folder)	
		bagfolder = logs_folder + "run_" + time.strftime("%Y-%m-%d-%H_%M") + "/"
		os.makedirs(bagfolder)

		self._joint_limits = self._joint_limits_rob if self.real_robot else self._joint_limits_sim

		# standard config
		joints_left_up = [0] * 6
		for i in range(6):
			joint_range = self._joint_limits[i][1] - self._joint_limits[i][0]
			joints_left_up[5-i] = self._joint_limits[i][0] + joint_range * 0.5
		joints_right_up = [0] * 6
		for i in range(6):
			joint_range = self._joint_limits[i+6][1] - self._joint_limits[i+6][0]
			joints_right_up[5-i] = self._joint_limits[i+6][0] + joint_range * 0.5
		_state_machine.userdata.joints_left_up = joints_left_up
		_state_machine.userdata.joints_right_up = joints_right_up
		
		rospy.loginfo('Average left joint positions: ' + ' '.join(map(str, joints_left_up)))		
		rospy.loginfo('Average right joint positions: ' + ' '.join(map(str, joints_right_up)))

		self._amplitudes = amplitudes
		self._frequencies = frequencies

		# [/MANUAL_CREATE]

		_sm_perform_sinusoide_motion = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['altered_gains', 'joint_index', 'traj_controller', 'nominal_gains'])

		with _sm_perform_sinusoide_motion:
			# x:64 y:39
			OperatableStateMachine.add('Set_Logfile_Name',
										FlexibleCalculationState(calculation=lambda i: bagfolder + self._traj_controllers[i[1]][1] + "_amp_" + str(i[0][0]) + "_freq_" + str(i[0][2]) + ".bag", input_keys=["gain_percentage", "joint_index"]),
										transitions={'done': 'Start_Gain_Logs'},
										autonomy={'done': Autonomy.Off},
										remapping={'gain_percentage': 'altered_gains', 'joint_index': 'joint_index', 'output_value': 'bagfile_name'})

			# x:332 y:61
			OperatableStateMachine.add('Start_Gain_Logs',
										StartRecordLogsState(topics_to_record=self.topics_to_record),
										transitions={'logging': 'Wait_For_Logging'},
										autonomy={'logging': Autonomy.Off},
										remapping={'bagfile_name': 'bagfile_name', 'rosbag_process': 'rosbag_process'})

			# x:154 y:397
			OperatableStateMachine.add('Stop_Gain_Logs',
										StopRecordLogsState(),
										transitions={'stopped': 'finished'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:344 y:344
			OperatableStateMachine.add('Reset_Joint_Gains',
										UpdateDynamicParameterState(param=gains_list),
										transitions={'updated': 'Stop_Gain_Logs', 'failed': 'failed'},
										autonomy={'updated': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'traj_controller': 'traj_controller', 'parameter_value': 'nominal_gains'})

			# x:564 y:89
			OperatableStateMachine.add('Wait_For_Logging',
										WaitState(wait_time=2),
										transitions={'done': 'Set_Joint_Gain'},
										autonomy={'done': Autonomy.Off})

			# x:598 y:361
			OperatableStateMachine.add('Wait_For_Motion',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Reset_Joint_Gains'},
										autonomy={'done': Autonomy.Off})

			# x:562 y:234
			OperatableStateMachine.add('Set_Joint_Gain',
										UpdateDynamicParameterState(param=gains_list),
										transitions={'updated': 'Wait_For_Motion', 'failed': 'failed'},
										autonomy={'updated': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'traj_controller': 'traj_controller', 'parameter_value': 'altered_gains'})


		_sm_move_joints_to_standard = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joints_right_up', 'joints_left_up'])

		with _sm_move_joints_to_standard:
			# x:71 y:145
			OperatableStateMachine.add('Move_Left_Arm_Back',
										MoveitMoveGroupState(planning_group="l_arm_group", joint_names=joint_names_left),
										transitions={'reached': 'Move_Right_Arm_Back', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_joint_config': 'joints_left_up'})

			# x:317 y:132
			OperatableStateMachine.add('Move_Right_Arm_Back',
										MoveitMoveGroupState(planning_group="r_arm_group", joint_names=joint_names_right),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_joint_config': 'joints_right_up'})


		_sm_test_individual_joint = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_index', 'traj_controller', 'none', 'joints_right_up', 'joints_left_up'])

		with _sm_test_individual_joint:
			# x:45 y:60
			OperatableStateMachine.add('Initialize_Iteration',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Move_Joints_To_Standard'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'iteration'})

			# x:176 y:388
			OperatableStateMachine.add('Decide_If_Tests_To_Go',
										DecisionState(outcomes=["done", "continue"], conditions=lambda it: "done" if it == (len(amplitudes) * len(frequencies)) else "continue"),
										transitions={'done': 'finished', 'continue': 'Calculate_Next_Gain_Value'},
										autonomy={'done': Autonomy.Off, 'continue': Autonomy.Off},
										remapping={'input_value': 'iteration'})

			# x:144 y:298
			OperatableStateMachine.add('Calculate_Next_Gain_Value',
										FlexibleCalculationState(calculation=self.calculate_gains, input_keys=["iteration", "nominal_gain"]),
										transitions={'done': 'Perform_Sinusoide_Motion'},
										autonomy={'done': Autonomy.High},
										remapping={'iteration': 'iteration', 'nominal_gain': 'nominal_gains', 'output_value': 'altered_gains'})

			# x:190 y:193
			OperatableStateMachine.add('Get_Joint_Gains',
										ReadDynamicParameterState(param=gains_list),
										transitions={'read': 'Calculate_Next_Gain_Value', 'failed': 'failed'},
										autonomy={'read': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'traj_controller': 'traj_controller', 'parameter_value': 'nominal_gains'})

			# x:158 y:505
			OperatableStateMachine.add('Increment_Iteration_Counter',
										CalculationState(calculation=lambda it: it + 1),
										transitions={'done': 'Decide_If_Tests_To_Go'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'iteration', 'output_value': 'iteration'})

			# x:210 y:53
			OperatableStateMachine.add('Move_Joints_To_Standard',
										_sm_move_joints_to_standard,
										transitions={'finished': 'Get_Joint_Gains', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joints_right_up': 'joints_right_up', 'joints_left_up': 'joints_left_up'})

			# x:496 y:393
			OperatableStateMachine.add('Perform_Sinusoide_Motion',
										_sm_perform_sinusoide_motion,
										transitions={'finished': 'Increment_Iteration_Counter', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'altered_gains': 'altered_gains', 'joint_index': 'joint_index', 'traj_controller': 'traj_controller', 'nominal_gains': 'nominal_gains'})


		_sm_test_joint_controls = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_index', 'none', 'joints_right_up', 'joints_left_up'])

		with _sm_test_joint_controls:
			# x:47 y:121
			OperatableStateMachine.add('Decide_Joints_To_Go',
										DecisionState(outcomes=["done", "continue"], conditions=lambda idx: "done" if idx == 12 else "continue"),
										transitions={'done': 'finished', 'continue': 'Set_Trajectory_Controller'},
										autonomy={'done': Autonomy.High, 'continue': Autonomy.Low},
										remapping={'input_value': 'joint_index'})

			# x:571 y:68
			OperatableStateMachine.add('Test_Individual_Joint',
										_sm_test_individual_joint,
										transitions={'finished': 'Increment_Joint_Index', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_index': 'joint_index', 'traj_controller': 'traj_controller', 'none': 'none', 'joints_right_up': 'joints_right_up', 'joints_left_up': 'joints_left_up'})

			# x:279 y:41
			OperatableStateMachine.add('Increment_Joint_Index',
										CalculationState(calculation=lambda it: it + 1),
										transitions={'done': 'Decide_Joints_To_Go'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joint_index', 'output_value': 'joint_index'})

			# x:447 y:224
			OperatableStateMachine.add('Set_Trajectory_Controller',
										CalculationState(calculation=lambda idx: self._traj_controllers[idx]),
										transitions={'done': 'Test_Individual_Joint'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joint_index', 'output_value': 'traj_controller'})



		with _state_machine:
			# x:112 y:38
			OperatableStateMachine.add('Check_Initial_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=False),
										transitions={'correct': 'Switch_To_Manipulate', 'incorrect': 'Set_Initial_Stand'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Low},
										remapping={'control_mode': 'control_mode'})

			# x:336 y:123
			OperatableStateMachine.add('Set_Initial_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Switch_To_Manipulate', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:60 y:235
			OperatableStateMachine.add('Switch_To_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Bring_Left_Arm_Up', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:105 y:428
			OperatableStateMachine.add('Bring_Left_Arm_Up',
										MoveitMoveGroupState(planning_group="l_arm_group", joint_names=joint_names_left),
										transitions={'reached': 'Bring_Right_Arm_Up', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_joint_config': 'joints_left_up'})

			# x:323 y:482
			OperatableStateMachine.add('Bring_Right_Arm_Up',
										MoveitMoveGroupState(planning_group="r_arm_group", joint_names=joint_names_right),
										transitions={'reached': 'Test_Joint_Controls', 'failed': 'failed'},
										autonomy={'reached': Autonomy.High, 'failed': Autonomy.High},
										remapping={'target_joint_config': 'joints_right_up'})

			# x:620 y:465
			OperatableStateMachine.add('Test_Joint_Controls',
										_sm_test_joint_controls,
										transitions={'finished': 'Change_Back_To_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_index': 'joint_index', 'none': 'none', 'joints_right_up': 'joints_right_up', 'joints_left_up': 'joints_left_up'})

			# x:887 y:469
			OperatableStateMachine.add('Change_Back_To_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Log_Finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:897 y:316
			OperatableStateMachine.add('Log_Finished',
										LogState(text="Behavior finished successfully!", severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def calculate_gains(self, input_values):
		i = input_values[0]
		nominal_gains = input_values[1]

		altered_gains = [0] * len(nominal_gains)
		altered_gains[0] = self._amplitudes[i%3]
		altered_gains[1] = 100
		altered_gains[2] = self._frequencies[i/3]
		
		return altered_gains
	# [/MANUAL_FUNC]
