#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_startup_unit_testing')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_atlas_states.check_current_control_mode_state import CheckCurrentControlModeState
from flexbe_states.start_record_logs_state import StartRecordLogsState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_states.stop_record_logs_state import StopRecordLogsState
from flexbe_atlas_states.execute_trajectory_state import ExecuteTrajectoryState
from vigir_behavior_praying_mantis_calibration.praying_mantis_calibration_sm import PrayingMantisCalibrationSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import time 	# for correct rosbag name
import rospy 	# for some debug prints
import os 		# for checking and creating directories
# [/MANUAL_IMPORT]


'''
Created on Tue Oct 28 2014
@author: Spyros and Philipp
'''
class StartupUnitTestingSM(Behavior):
	'''
	Version of the Startup Unit Testing behavior which has been created using the Behavior Editor
	'''


	def __init__(self):
		super(StartupUnitTestingSM, self).__init__()
		self.name = 'Startup Unit Testing'

		# parameters of this behavior
		self.add_parameter('topics_to_record', '')
		self.add_parameter('real_robot', True)

		# references to used behaviors
		self.add_behavior(PrayingMantisCalibrationSM, 'Praying Mantis Calibration')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]


# JOINT LIMITS (different for sim and real robot ?)
#
# 0-5	left arm
# 6-11	right arm
# for each: wrx, wry, elx, ely, shx, shz

#TODO: Refactor limits' format to match "Praying Mantis" one

# simulation (new)
		self._joint_limits_sim = [ \
			[-1.1781, +1.1781], \
			[0.0, +3.14159], \
			[0.00, +2.35619], \
			[0.00, +3.14159], \
			[-1.5708, +1.5708], \
			[-1.5708, +0.785398], \
			 \
			[-1.1781, +1.1781], \
			[0.0, +3.14159], \
			[-2.35619, 0.00], \
			[0.00, +3.14159], \
			[-1.5708, +1.5708], \
			[-0.785398, +1.5708], \
		]
# real robot (old)
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

# right arm basic config
#joints = [0.0223725773394 0.0318480841815 1.605915308 -0.0668966025114 0.00500942254439 0.00276394281536]
#r_arm_*: shz, shx, ely, elx, wry, wrx

# in basic config
#r_arm_wrx: -1.57  ---  0.44			| total: 2.01 -->  -1.37  ---   0.24
#r_arm_wry: -1.57  ---  1.57			| total: 3.14 -->  -1.26  ---   1.26
#r_arm_elx: -2.35 (1)  ---  0.00 		| total: 2.35 -->  -2.12  ---  -0.23	(1)
#r_arm_ely:  0.00  ---  3.14			| total: 3.14 -->   0.31  ---   2.83
# risky:
#r_arm_shx: -1.75 (2)  ---  1.40 (3)	| total: 3.15 -->  -1.44  ---   1.09 	(2)(3)
#r_arm_shz: -1.96 (4)  ---  1.96 (4)	| total: 3.92 -->  -1.57  ---   1.57	(4)(4)


#(1) gets very close to the robot, be careful and set r_arm_shz to 0.76
#(2) watch out for head, set r_arm_shz to 1.61
#(3) watch out for self collision, set r_arm_shz to 0.84
#(4) arm stretched out to the top(!), set r_arm_shx to -0.86

# FOR LEFT:
# x is mirrored, y the same


		# set joint configurations to test
		self._joint_configs_right = [ \
			[0.02237, 0.03185, 1.60592, -0.06690, 0.00501, -1.37], \
			[0.02237, 0.03185, 1.60592, -0.06690, 0.00501, 0.24], \
			\
			[0.02237, 0.03185, 1.60592, -0.06690, -1.26, 0.00276], \
			[0.02237, 0.03185, 1.60592, -0.06690, 1.26, 0.00276], \
			\
			[0.76, 0.03185, 1.60592, -2.12, 0.00501, 0.00276], \
			[0.02237, 0.03185, 1.60592, -0.23, 0.00501, 0.00276], \
			\
			[0.02237, 0.03185, 0.31, -0.06690, 0.00501, 0.00276], \
			[0.02237, 0.03185, 2.83, -0.06690, 0.00501, 0.00276], \
			\
			[1.61, -1.44, 1.60592, -0.06690, 0.00501, 0.00276], \
			[0.84, 1.09, 1.60592, -0.06690, 0.00501, 0.00276], \
			\
			[-1.57, -0.86, 1.60592, -0.06690, 0.00501, 0.00276], \
			[1.57, -0.86, 1.60592, -0.06690, 0.00501, 0.00276], \
		]
		self._joint_configs_left = [ \
			[0.02237, -0.03185, 1.60592, 0.06690, 0.00501, 1.37], \
			[0.02237, -0.03185, 1.60592, 0.06690, 0.00501, -0.24], \
			\
			[0.02237, -0.03185, 1.60592, 0.06690, -1.26, -0.00276], \
			[0.02237, -0.03185, 1.60592, 0.06690, 1.26, -0.00276], \
			\
			[0.76, -0.03185, 1.60592, 2.12, 0.00501, -0.00276], \
			[0.02237, -0.03185, 1.60592, 0.23, 0.00501, -0.00276], \
			\
			[0.02237, -0.03185, 0.31, 0.06690, 0.00501, -0.00276], \
			[0.02237, -0.03185, 2.83, 0.06690, 0.00501, -0.00276], \
			\
			[1.61, 1.44, 1.60592, 0.06690, 0.00501, -0.00276], \
			[0.84, -1.09, 1.60592, 0.06690, 0.00501, -0.00276], \
			\
			[-1.57, 0.86, 1.60592, 0.06690, 0.00501, -0.00276], \
			[1.57, 0.86, 1.60592, 0.06690, 0.00501, -0.00276], \
		]


		# define given trajectories
		self._traj_right_1 = [ \
			[0.3, 1.3, 2.0, -0.5, 0.0, -0.0], \
			[0.78, 0.8, 2.0, -2.0, 0.0, -0.6], \
			[0.0, 0.8, 1.7, -2.0, 0.0, -0.8], \
			[0.0, 0.2, 3.0, -2.1, 0.0, 0.8], \
			[0.3, 1.3, 2.0, -0.5, 0.0, -0.0], \
		]
		self._times_right_1 = [3, 6, 9, 12, 15]

		self._traj_left_1 = [ \
			[0.3, -1.3, 2.0, 0.5, 0.0, 0.0], \
			[0.78, -0.8, 2.0, 2.0, 0.0, 0.6], \
			[0.0, -0.8, 1.7, 2.0, 0.0, 0.8], \
			[0.0, -0.2, 3.0, 2.1, 0.0, -0.8], \
			[0.3, -1.3, 2.0, 0.5, 0.0, 0.0], \
		]
		self._times_left_1 = [3, 6, 9, 12, 15]

		# [/MANUAL_INIT]


	def create(self):
		joint_names_right = ["r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx"]
		joint_names_left = ["l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx"]
		# x:1092 y:395, x:898 y:121
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_joint_config = []		
		_state_machine.userdata.joint_config_index = 0
		_state_machine.userdata.basic_joint_config_right = [] # calculated
		_state_machine.userdata.basic_joint_config_left = [] # calculated
		_state_machine.userdata.predefined_traj_left = self._traj_left_1
		_state_machine.userdata.predefined_times_left = self._times_left_1		
		_state_machine.userdata.predefined_traj_right = self._traj_right_1
		_state_machine.userdata.predefined_times_right = self._times_right_1		
		_state_machine.userdata.bagfile = ""		

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		logs_folder = os.path.expanduser('~/startup_logs/')
		if not os.path.exists(logs_folder):
			os.makedirs(logs_folder)
		_state_machine.userdata.bagfile = '~/startup_logs/startup_logs_' + time.strftime("%Y-%m-%d-%H:%M") + '.bag'

		self._joint_limits = self._joint_limits_rob if self.real_robot else self._joint_limits_sim

		# standard config
		basic_joint_config_left = [0] * 6
		for i in range(6):
			joint_range = self._joint_limits[i][1] - self._joint_limits[i][0]
			basic_joint_config_left[5-i] = self._joint_limits[i][0] + joint_range * 0.5
		basic_joint_config_right = [0] * 6
		for i in range(6):
			joint_range = self._joint_limits[i+6][1] - self._joint_limits[i+6][0]
			basic_joint_config_right[5-i] = self._joint_limits[i+6][0] + joint_range * 0.5
		_state_machine.userdata.basic_joint_config_left = basic_joint_config_left
		_state_machine.userdata.basic_joint_config_right = basic_joint_config_right

		# left
		self._joint_configs_left = []
		for i in range(6):
			joint_config_up = list(_state_machine.userdata.basic_joint_config_left)
			joint_config_down = list(_state_machine.userdata.basic_joint_config_left)
			joint_range = self._joint_limits[i][1] - self._joint_limits[i][0]
			joint_config_up[5-i] = self._joint_limits[i][0] + joint_range * 0.9
			joint_config_down[5-i] = self._joint_limits[i][0] + joint_range * 0.1
			self._joint_configs_left.append(joint_config_up)
			self._joint_configs_left.append(joint_config_down)
		# right
		self._joint_configs_right = []
		for i in range(6):
			joint_config_up = list(_state_machine.userdata.basic_joint_config_left)
			joint_config_down = list(_state_machine.userdata.basic_joint_config_left)
			joint_range = self._joint_limits[i+6][1] - self._joint_limits[i+6][0]
			joint_config_up[5-i] = self._joint_limits[i+6][0] + joint_range * 0.9
			joint_config_down[5-i] = self._joint_limits[i+6][0] + joint_range * 0.1
			self._joint_configs_right.append(joint_config_up)
			self._joint_configs_right.append(joint_config_down)

		# [/MANUAL_CREATE]

		# x:604 y:158, x:428 y:80
		_sm_execute_predefined_trajectories_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['predefined_traj_left', 'predefined_times_left', 'predefined_traj_right', 'predefined_times_right'])

		with _sm_execute_predefined_trajectories_0:
			# x:52 y:73
			OperatableStateMachine.add('Execute_Trajectory_Right',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_RIGHT_ARM, joint_names=joint_names_right),
										transitions={'done': 'Execute_Trajectory_Left', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_positions': 'predefined_traj_right', 'time': 'predefined_times_right'})

			# x:374 y:151
			OperatableStateMachine.add('Execute_Trajectory_Left',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_LEFT_ARM, joint_names=joint_names_left),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.High},
										remapping={'joint_positions': 'predefined_traj_left', 'time': 'predefined_times_left'})



		with _state_machine:
			# x:47 y:63
			OperatableStateMachine.add('Check_Control_Mode_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=False),
										transitions={'correct': 'Start_Logging', 'incorrect': 'Set_Control_Mode_To_Stand'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Low},
										remapping={'control_mode': 'control_mode'})

			# x:69 y:165
			OperatableStateMachine.add('Start_Logging',
										StartRecordLogsState(topics_to_record=self.topics_to_record),
										transitions={'logging': 'Set_Control_Mode_To_Manipulate'},
										autonomy={'logging': Autonomy.Off},
										remapping={'bagfile_name': 'bagfile', 'rosbag_process': 'rosbag_process'})

			# x:36 y:269
			OperatableStateMachine.add('Set_Control_Mode_To_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Execute_Predefined_Trajectories', 'failed': 'Stop_Logging_When_Failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:579 y:390
			OperatableStateMachine.add('Stop_Logging',
										StopRecordLogsState(),
										transitions={'stopped': 'Back_To_Stand_When_Finished'},
										autonomy={'stopped': Autonomy.High},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:308 y:114
			OperatableStateMachine.add('Set_Control_Mode_To_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Start_Logging', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:28 y:384
			OperatableStateMachine.add('Execute_Predefined_Trajectories',
										_sm_execute_predefined_trajectories_0,
										transitions={'finished': 'Praying Mantis Calibration', 'failed': 'Stop_Logging_When_Failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'predefined_traj_left': 'predefined_traj_left', 'predefined_times_left': 'predefined_times_left', 'predefined_traj_right': 'predefined_traj_right', 'predefined_times_right': 'predefined_times_right'})

			# x:328 y:269
			OperatableStateMachine.add('Stop_Logging_When_Failed',
										StopRecordLogsState(),
										transitions={'stopped': 'Back_To_Stand_When_Failed'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:654 y:269
			OperatableStateMachine.add('Back_To_Stand_When_Failed',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'failed', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:819 y:387
			OperatableStateMachine.add('Back_To_Stand_When_Finished',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.Off})

			# x:319 y:383
			OperatableStateMachine.add('Praying Mantis Calibration',
										self.use_behavior(PrayingMantisCalibrationSM, 'Praying Mantis Calibration'),
										transitions={'finished': 'Stop_Logging', 'failed': 'Stop_Logging_When_Failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
