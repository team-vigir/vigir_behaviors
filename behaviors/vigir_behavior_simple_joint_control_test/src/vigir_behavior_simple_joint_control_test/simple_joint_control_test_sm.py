#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_simple_joint_control_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from vigir_flexbe_states.check_current_control_mode_state import CheckCurrentControlModeState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.moveit_move_group_state import MoveitMoveGroupState
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.wait_state import WaitState
from vigir_flexbe_states.execute_trajectory_state import ExecuteTrajectoryState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from vigir_flexbe_states.update_dynamic_parameter_state import UpdateDynamicParameterState
from vigir_flexbe_states.read_dynamic_parameter_state import ReadDynamicParameterState
from flexbe_states.start_record_logs_state import StartRecordLogsState
from flexbe_states.stop_record_logs_state import StopRecordLogsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import time
import os
import rospy
# [/MANUAL_IMPORT]


'''
Created on Mon Nov 03 2014
@author: Philipp and Spyros
'''
class SimpleJointControlTestSM(Behavior):
	'''
	Get step response of joint controllers by varying PID gains.
	'''


	def __init__(self):
		super(SimpleJointControlTestSM, self).__init__()
		self.name = 'Simple Joint Control Test'

		# parameters of this behavior
		self.add_parameter('topics_to_record', '')
		self.add_parameter('joint_upper_bounds', 0.6)
		self.add_parameter('joint_lower_bounds', 0.4)
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
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joint_names_left = ["l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx"]
		joint_names_right = ["r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx"]
		wait_time = 3.0
		bagfolder = "" # calculated
		gains_list = {'pid_gains': ['p', 'i', 'd'], 'bdi_gains': ['k_qd_p', 'ff_qd_d'], 'vigir_gains': ['ff_bang', 'ff_effort', 'ff_friction']}
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joints_left_up = [] # calculated
		_state_machine.userdata.joints_right_up = [] # calculated
		_state_machine.userdata.joint_index = 0
		_state_machine.userdata.zero_time = [0.02]
		_state_machine.userdata.joint_positions_up = [] # calculated
		_state_machine.userdata.joint_positions_down = [] # calculated
		_state_machine.userdata.joint_index = 0
		_state_machine.userdata.none = None
		_state_machine.userdata.init_time = [3.0]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# 'Basic' configuration for SIMULATION
		#_state_machine.userdata.joints_left_up = [0.00, 0.18, 1.57, 1.18, 0.00, 0.57]
		#_state_machine.userdata.joints_right_up = [0.00, -0.18, 1.57, -1.18, 0.00, -0.57]

		logs_folder = os.path.expanduser('~/joint_control_tests/')
		if not os.path.exists(logs_folder):
			os.makedirs(logs_folder)
		bagfolder = os.path.join(logs_folder, "run_" + time.strftime("%Y-%m-%d-%H_%M"))
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
		
		# left
		for i in range(6):
			joint_config_up = list(_state_machine.userdata.joints_left_up)
			joint_config_down = list(_state_machine.userdata.joints_left_up)
			joint_range = self._joint_limits[i][1] - self._joint_limits[i][0]
			joint_config_up[5-i] = self._joint_limits[i][0] + joint_range * self.joint_upper_bounds
			joint_config_down[5-i] = self._joint_limits[i][0] + joint_range * self.joint_lower_bounds
			self._joint_configs_up.append([joint_config_up])
			self._joint_configs_down.append([joint_config_down])
			rospy.loginfo('Left Joint Config Up: ' + ' '.join(map(str, joint_config_up)))
			rospy.loginfo('Left Joint Config Dn: ' + ' '.join(map(str, joint_config_down)))
		# right
		for i in range(6):
			joint_config_up = list(_state_machine.userdata.joints_right_up)
			joint_config_down = list(_state_machine.userdata.joints_right_up)
			joint_range = self._joint_limits[i+6][1] - self._joint_limits[i+6][0]
			joint_config_up[5-i] = self._joint_limits[i+6][0] + joint_range * self.joint_upper_bounds
			joint_config_down[5-i] = self._joint_limits[i+6][0] + joint_range * self.joint_lower_bounds
			self._joint_configs_up.append([joint_config_up])
			self._joint_configs_down.append([joint_config_down])
			rospy.loginfo('Right Joint Config Up: ' + ' '.join(map(str, joint_config_up)))
			rospy.loginfo('Right Joint Config Dn: ' + ' '.join(map(str, joint_config_down)))
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_move_joint_down_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_index', 'joint_positions_down', 'zero_time', 'joints_right_up', 'joints_left_up', 'init_time'])

		with _sm_move_joint_down_0:
			# x:71 y:145
			OperatableStateMachine.add('Move_Left_Arm_Back',
										MoveitMoveGroupState(planning_group="l_arm_group", joint_names=joint_names_left),
										transitions={'reached': 'Move_Right_Arm_Back', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_joint_config': 'joints_left_up'})

			# x:639 y:69
			OperatableStateMachine.add('Move_Left_Joint_Down',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_LEFT_ARM, joint_names=joint_names_left),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_down', 'time': 'init_time'})

			# x:631 y:200
			OperatableStateMachine.add('Move_Right_Joint_Down',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_RIGHT_ARM, joint_names=joint_names_right),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_down', 'time': 'init_time'})

			# x:201 y:54
			OperatableStateMachine.add('Move_Right_Arm_Back',
										MoveitMoveGroupState(planning_group="r_arm_group", joint_names=joint_names_right),
										transitions={'reached': 'Decide_Left_Or_Right', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_joint_config': 'joints_right_up'})

			# x:429 y:62
			OperatableStateMachine.add('Decide_Left_Or_Right',
										DecisionState(outcomes=["left", "right"], conditions=lambda it: "left" if it < 6 else "right"),
										transitions={'left': 'Move_Left_Joint_Down', 'right': 'Move_Right_Joint_Down'},
										autonomy={'left': Autonomy.High, 'right': Autonomy.High},
										remapping={'input_value': 'joint_index'})


		# x:30 y:365, x:130 y:365
		_sm_perform_gain_test_right_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_positions_up', 'joint_positions_down', 'zero_time'])

		with _sm_perform_gain_test_right_1:
			# x:84 y:39
			OperatableStateMachine.add('Initial_Wait',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perform_Step_Up'},
										autonomy={'done': Autonomy.Off})

			# x:80 y:218
			OperatableStateMachine.add('Wait_Up',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perform_Step_Down'},
										autonomy={'done': Autonomy.Off})

			# x:44 y:331
			OperatableStateMachine.add('Perform_Step_Down',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_RIGHT_ARM, joint_names=joint_names_right),
										transitions={'done': 'Wait_Down', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_down', 'time': 'zero_time'})

			# x:73 y:440
			OperatableStateMachine.add('Wait_Down',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perforn_Step_Up_2'},
										autonomy={'done': Autonomy.Off})

			# x:414 y:401
			OperatableStateMachine.add('Perforn_Step_Up_2',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_RIGHT_ARM, joint_names=joint_names_right),
										transitions={'done': 'Wait_Up_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_up', 'time': 'zero_time'})

			# x:442 y:291
			OperatableStateMachine.add('Wait_Up_2',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perform_Step_Down_2'},
										autonomy={'done': Autonomy.Off})

			# x:416 y:167
			OperatableStateMachine.add('Perform_Step_Down_2',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_RIGHT_ARM, joint_names=joint_names_right),
										transitions={'done': 'Wait_Down_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_down', 'time': 'zero_time'})

			# x:449 y:62
			OperatableStateMachine.add('Wait_Down_2',
										WaitState(wait_time=wait_time),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:48 y:113
			OperatableStateMachine.add('Perform_Step_Up',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_RIGHT_ARM, joint_names=joint_names_right),
										transitions={'done': 'Wait_Up', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_up', 'time': 'zero_time'})


		# x:30 y:365, x:130 y:365
		_sm_perform_gain_test_left_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_positions_up', 'joint_positions_down', 'zero_time'])

		with _sm_perform_gain_test_left_2:
			# x:84 y:39
			OperatableStateMachine.add('Initial_Wait',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perform_Step_Up_1'},
										autonomy={'done': Autonomy.Off})

			# x:87 y:232
			OperatableStateMachine.add('Wait_Up_1',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perform_Step_Down_1'},
										autonomy={'done': Autonomy.Off})

			# x:50 y:321
			OperatableStateMachine.add('Perform_Step_Down_1',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_LEFT_ARM, joint_names=joint_names_left),
										transitions={'done': 'Wait_Down_1', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_down', 'time': 'zero_time'})

			# x:77 y:415
			OperatableStateMachine.add('Wait_Down_1',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perforn_Step_Up_2'},
										autonomy={'done': Autonomy.Off})

			# x:51 y:131
			OperatableStateMachine.add('Perform_Step_Up_1',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_LEFT_ARM, joint_names=joint_names_left),
										transitions={'done': 'Wait_Up_1', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_up', 'time': 'zero_time'})

			# x:414 y:401
			OperatableStateMachine.add('Perforn_Step_Up_2',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_LEFT_ARM, joint_names=joint_names_left),
										transitions={'done': 'Wait_Up_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_up', 'time': 'zero_time'})

			# x:442 y:291
			OperatableStateMachine.add('Wait_Up_2',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Perform_Step_Down_2'},
										autonomy={'done': Autonomy.Off})

			# x:416 y:167
			OperatableStateMachine.add('Perform_Step_Down_2',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_LEFT_ARM, joint_names=joint_names_left),
										transitions={'done': 'Wait_Down_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions_down', 'time': 'zero_time'})

			# x:449 y:62
			OperatableStateMachine.add('Wait_Down_2',
										WaitState(wait_time=wait_time),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_test_individual_joint_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_positions_up', 'joint_positions_down', 'joint_index', 'traj_controller', 'none', 'zero_time', 'joints_right_up', 'joints_left_up', 'init_time'])

		with _sm_test_individual_joint_3:
			# x:45 y:60
			OperatableStateMachine.add('Initialize_Iteration',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Move_Joint_Down'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'iteration'})

			# x:520 y:555
			OperatableStateMachine.add('Perform_Gain_Test_Left',
										_sm_perform_gain_test_left_2,
										transitions={'finished': 'Stop_Gain_Logs', 'failed': 'Stop_Gain_Logs'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_positions_up': 'joint_positions_up', 'joint_positions_down': 'joint_positions_down', 'zero_time': 'zero_time'})

			# x:176 y:388
			OperatableStateMachine.add('Decide_If_Tests_To_Go',
										DecisionState(outcomes=["done", "continue"], conditions=lambda it: "done" if it == 5 else "continue"),
										transitions={'done': 'Reset_Joint_Gains', 'continue': 'Calculate_Next_Gain_Value'},
										autonomy={'done': Autonomy.Off, 'continue': Autonomy.Off},
										remapping={'input_value': 'iteration'})

			# x:144 y:298
			OperatableStateMachine.add('Calculate_Next_Gain_Value',
										FlexibleCalculationState(calculation=self.calculate_gains, input_keys=["iteration", "nominal_gain"]),
										transitions={'done': 'Set_Joint_Gain'},
										autonomy={'done': Autonomy.Off},
										remapping={'iteration': 'iteration', 'nominal_gain': 'nominal_gains', 'output_value': 'altered_gains'})

			# x:395 y:268
			OperatableStateMachine.add('Set_Joint_Gain',
										UpdateDynamicParameterState(param=gains_list),
										transitions={'updated': 'Set_Logfile_Name', 'failed': 'failed'},
										autonomy={'updated': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'traj_controller': 'traj_controller', 'parameter_value': 'altered_gains'})

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

			# x:798 y:435
			OperatableStateMachine.add('Decide_Left_Or_Right',
										DecisionState(outcomes=["left", "right"], conditions=lambda it: "left" if it < 6 else "right"),
										transitions={'left': 'Perform_Gain_Test_Left', 'right': 'Perform_Gain_Test_Right'},
										autonomy={'left': Autonomy.High, 'right': Autonomy.High},
										remapping={'input_value': 'joint_index'})

			# x:811 y:624
			OperatableStateMachine.add('Perform_Gain_Test_Right',
										_sm_perform_gain_test_right_1,
										transitions={'finished': 'Stop_Gain_Logs', 'failed': 'Stop_Gain_Logs'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_positions_up': 'joint_positions_up', 'joint_positions_down': 'joint_positions_down', 'zero_time': 'zero_time'})

			# x:545 y:458
			OperatableStateMachine.add('Start_Gain_Logs',
										StartRecordLogsState(topics_to_record=self.topics_to_record),
										transitions={'logging': 'Decide_Left_Or_Right'},
										autonomy={'logging': Autonomy.Off},
										remapping={'bagfile_name': 'bagfile_name', 'rosbag_process': 'rosbag_process'})

			# x:184 y:616
			OperatableStateMachine.add('Stop_Gain_Logs',
										StopRecordLogsState(),
										transitions={'stopped': 'Increment_Iteration_Counter'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:576 y:346
			OperatableStateMachine.add('Set_Logfile_Name',
										FlexibleCalculationState(calculation=lambda i: bagfolder + self._traj_controllers[i[1]][1] + "_k_p_" + str(i[0][0]) + ".bag", input_keys=["gain_percentage", "joint_index"]),
										transitions={'done': 'Start_Gain_Logs'},
										autonomy={'done': Autonomy.Off},
										remapping={'gain_percentage': 'altered_gains', 'joint_index': 'joint_index', 'output_value': 'bagfile_name'})

			# x:210 y:53
			OperatableStateMachine.add('Move_Joint_Down',
										_sm_move_joint_down_0,
										transitions={'finished': 'Get_Joint_Gains', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_index': 'joint_index', 'joint_positions_down': 'joint_positions_down', 'zero_time': 'zero_time', 'joints_right_up': 'joints_right_up', 'joints_left_up': 'joints_left_up', 'init_time': 'init_time'})

			# x:365 y:430
			OperatableStateMachine.add('Reset_Joint_Gains',
										UpdateDynamicParameterState(param=gains_list),
										transitions={'updated': 'finished', 'failed': 'failed'},
										autonomy={'updated': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'traj_controller': 'traj_controller', 'parameter_value': 'nominal_gains'})


		# x:30 y:365, x:130 y:365
		_sm_test_joint_controls_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_index', 'none', 'zero_time', 'joints_right_up', 'joints_left_up', 'init_time'])

		with _sm_test_joint_controls_4:
			# x:47 y:121
			OperatableStateMachine.add('Decide_Joints_To_Go',
										DecisionState(outcomes=["done", "continue"], conditions=lambda idx: "done" if idx == len(self._joint_configs_down) else "continue"),
										transitions={'done': 'finished', 'continue': 'Select_Next_Joint_Up'},
										autonomy={'done': Autonomy.High, 'continue': Autonomy.Low},
										remapping={'input_value': 'joint_index'})

			# x:257 y:290
			OperatableStateMachine.add('Select_Next_Joint_Up',
										CalculationState(calculation=lambda idx: self._joint_configs_up[idx]),
										transitions={'done': 'Select_Next_Joint_Down'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joint_index', 'output_value': 'joint_positions_up'})

			# x:571 y:68
			OperatableStateMachine.add('Test_Individual_Joint',
										_sm_test_individual_joint_3,
										transitions={'finished': 'Increment_Joint_Index', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_positions_up': 'joint_positions_up', 'joint_positions_down': 'joint_positions_down', 'joint_index': 'joint_index', 'traj_controller': 'traj_controller', 'none': 'none', 'zero_time': 'zero_time', 'joints_right_up': 'joints_right_up', 'joints_left_up': 'joints_left_up', 'init_time': 'init_time'})

			# x:529 y:324
			OperatableStateMachine.add('Select_Next_Joint_Down',
										CalculationState(calculation=lambda idx: self._joint_configs_down[idx]),
										transitions={'done': 'Set_Trajectory_Controller'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joint_index', 'output_value': 'joint_positions_down'})

			# x:222 y:51
			OperatableStateMachine.add('Increment_Joint_Index',
										CalculationState(calculation=lambda it: it + 1),
										transitions={'done': 'Decide_Joints_To_Go'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joint_index', 'output_value': 'joint_index'})

			# x:559 y:189
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
										_sm_test_joint_controls_4,
										transitions={'finished': 'Change_Back_To_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_index': 'joint_index', 'none': 'none', 'zero_time': 'zero_time', 'joints_right_up': 'joints_right_up', 'joints_left_up': 'joints_left_up', 'init_time': 'init_time'})

			# x:831 y:349
			OperatableStateMachine.add('Change_Back_To_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def calculate_gains(self, input_values):
		iteration = input_values[0]
		nominal_gains = input_values[1]
		gain_percentage = nominal_gains[0] * (0.4 + 0.2 * iteration)
		altered_gains = [gain_percentage]
		for gain in nominal_gains[1:]:
			altered_gains.append(0)
		return altered_gains
	# [/MANUAL_FUNC]
