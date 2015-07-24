#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_force_torque_sensor_calibration')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from flexbe_atlas_states.calculate_force_torque_calibration_state import CalculateForceTorqueCalibration
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from flexbe_atlas_states.update_dynamic_parameter_impedance_controller_state import UpdateDynamicParameterImpedanceControllerState
from flexbe_states.calculation_state import CalculationState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_atlas_states.video_logging_state import VideoLoggingState
from flexbe_states.stop_record_logs_state import StopRecordLogsState
from flexbe_states.wait_state import WaitState
from flexbe_atlas_states.execute_trajectory_whole_body_state import ExecuteTrajectoryWholeBodyState
from flexbe_states.start_record_logs_state import StartRecordLogsState
from flexbe_atlas_states.moveit_move_group_plan_state import MoveItMoveGroupPlanState
from flexbe_atlas_states.generate_trajectory_from_txtfile_state import GenerateTrajectoryFromTxtfileState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import os
import time
import pprint

import rospy
from control_msgs.msg import *
from trajectory_msgs.msg import *

from flexbe_core.proxy import ProxyPublisher

from vigir_flexbe_behaviors.atlas_definitions import AtlasDefinitions
from vigir_flexbe_behaviors.atlas_functions import AtlasFunctions
# [/MANUAL_IMPORT]


'''
Created on Sun Apr 26 2015
@author: Moritz Schappler
'''
class ForceTorqueSensorCalibrationSM(Behavior):
	'''
	A behavior that moves the arms and collects data for calibrating the force torque sensors.
	'''


	def __init__(self):
		super(ForceTorqueSensorCalibrationSM, self).__init__()
		self.name = 'Force Torque Sensor Calibration'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		self._offset_topic = "/flor/controller/encoder_offsets"
		self._pub =  ProxyPublisher({self._offset_topic: JointTrajectory})
		self.topics_to_record = '/flor/controller/atlas_state, /flor/controller/joint_command, /tf'
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 47 211 /Perform_Checks/Manipulate_Limits
		# Without this output_key, Check Behavior complains. Because traj_past_limits could in theory be undefined during runtime.



	def create(self):
		warn_threshold = 0.1 # for joint offsets
		bag_folder_out = "~/ft_calib/ft_logs"
		initial_mode = "stand"
		motion_mode = "manipulate"
		transitiontime = 0.5
		settlingtime = 0.5
		txtfile_name_left_arm = "~/ft_calib/input/SI_E047_FT_Calib_Arms_l_arm.txt"
		txtfile_name_right_arm = "~/ft_calib/input/SI_E047_FT_Calib_Arms_r_arm.txt"
		calibration_chain = ["right_arm"]
		static_calibration_data = {}
		# x:783 y:13, x:649 y:73
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.parameter_keys_dict = None
		_state_machine.userdata.calibration_chain_user = calibration_chain
		_state_machine.userdata.txtfile_name_left_arm_user = txtfile_name_left_arm
		_state_machine.userdata.txtfile_name_right_arm_user = txtfile_name_right_arm

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		bag_folder_out = os.path.expanduser(bag_folder_out)
		if not os.path.exists(bag_folder_out):
			os.makedirs(bag_folder_out)

		# Create STAND posture trajectory
		_state_machine.userdata.stand_posture = AtlasFunctions.gen_stand_posture_trajectory()

		# [/MANUAL_CREATE]

		# x:861 y:31, x:1047 y:103
		_sm_starting_point_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['experiment_name', 'trajectories'])

		with _sm_starting_point_0:
			# x:49 y:42
			OperatableStateMachine.add('Gen_Starting_Name',
										CalculationState(calculation=lambda en: en + "_starting"),
										transitions={'done': 'Gen_Starting_Bagfile_Name'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'experiment_name', 'output_value': 'starting_name'})

			# x:42 y:231
			OperatableStateMachine.add('Record_Starting_Point',
										StartRecordLogsState(topics_to_record=self.topics_to_record),
										transitions={'logging': 'Wait_for_Rosbag_Record'},
										autonomy={'logging': Autonomy.Off},
										remapping={'bagfile_name': 'output_bagfile_starting', 'rosbag_process': 'rosbag_process_starting'})

			# x:38 y:330
			OperatableStateMachine.add('Wait_for_Rosbag_Record',
										WaitState(wait_time=1.0),
										transitions={'done': 'Extract_Left_Arm_Part'},
										autonomy={'done': Autonomy.Off})

			# x:29 y:133
			OperatableStateMachine.add('Gen_Starting_Bagfile_Name',
										CalculationState(calculation=lambda en: os.path.join(bag_folder_out, en) + ".bag"),
										transitions={'done': 'Record_Starting_Point'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'starting_name', 'output_value': 'output_bagfile_starting'})

			# x:536 y:47
			OperatableStateMachine.add('Stop_Recording_Starting_Point',
										StopRecordLogsState(),
										transitions={'stopped': 'finished'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process_starting'})

			# x:228 y:757
			OperatableStateMachine.add('Plan_to_Starting_Point_Left_Arm',
										MoveItMoveGroupPlanState(vel_scaling=0.1),
										transitions={'done': 'Plan_to_Starting_Point_Right_Arm', 'failed': 'Report_Starting_Point_Failure'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'desired_goal': 'trajectories_left_arm', 'plan_to_goal': 'plan_to_goal_left_arm'})

			# x:236 y:655
			OperatableStateMachine.add('Plan_to_Starting_Point_Right_Arm',
										MoveItMoveGroupPlanState(vel_scaling=0.1),
										transitions={'done': 'Plan_to_Starting_Point_Left_Leg', 'failed': 'Report_Starting_Point_Failure'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'desired_goal': 'trajectories_right_arm', 'plan_to_goal': 'plan_to_goal_right_arm'})

			# x:272 y:47
			OperatableStateMachine.add('Go_to_Starting_Point',
										ExecuteTrajectoryWholeBodyState(controllers=[]),
										transitions={'done': 'Stop_Recording_Starting_Point', 'failed': 'Report_Starting_Point_Failure'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'trajectories': 'trajectories_all'})

			# x:536 y:169
			OperatableStateMachine.add('Report_Starting_Point_Failure',
										LogState(text="Failed to plan or go to starting point!", severity=Logger.REPORT_WARN),
										transitions={'done': 'Stop_Recording_When_Failed'},
										autonomy={'done': Autonomy.Full})

			# x:34 y:424
			OperatableStateMachine.add('Extract_Left_Arm_Part',
										CalculationState(calculation=lambda t: {'left_arm': t['left_arm']} if 'left_arm' in t else None),
										transitions={'done': 'Extract_Right_Arm_Part'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'trajectories', 'output_value': 'trajectories_left_arm'})

			# x:21 y:507
			OperatableStateMachine.add('Extract_Right_Arm_Part',
										CalculationState(calculation=lambda t: {'right_arm': t['right_arm']} if 'right_arm' in t else None),
										transitions={'done': 'Extract_Left_Leg_Part'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'trajectories', 'output_value': 'trajectories_right_arm'})

			# x:293 y:146
			OperatableStateMachine.add('Combine_Plans',
										FlexibleCalculationState(calculation=self.combine_plans, input_keys=['left_arm', 'right_arm', 'left_leg', 'right_leg', 'torso']),
										transitions={'done': 'Go_to_Starting_Point'},
										autonomy={'done': Autonomy.Low},
										remapping={'left_arm': 'plan_to_goal_left_arm', 'right_arm': 'plan_to_goal_right_arm', 'left_leg': 'plan_to_goal_left_leg', 'right_leg': 'plan_to_goal_right_leg', 'torso': 'plan_to_goal_torso', 'output_value': 'trajectories_all'})

			# x:789 y:167
			OperatableStateMachine.add('Stop_Recording_When_Failed',
										StopRecordLogsState(),
										transitions={'stopped': 'failed'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process_starting'})

			# x:24 y:601
			OperatableStateMachine.add('Extract_Left_Leg_Part',
										CalculationState(calculation=lambda t: {'left_leg': t['left_leg']} if 'left_leg' in t else None),
										transitions={'done': 'Extract_Right_Leg_Part'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'trajectories', 'output_value': 'trajectories_left_leg'})

			# x:22 y:665
			OperatableStateMachine.add('Extract_Right_Leg_Part',
										CalculationState(calculation=lambda t: {'right_leg': t['right_leg']} if 'right_leg' in t else None),
										transitions={'done': 'Extract_Torso_Part'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'trajectories', 'output_value': 'trajectories_right_leg'})

			# x:33 y:765
			OperatableStateMachine.add('Extract_Torso_Part',
										CalculationState(calculation=lambda t: {'torso': t['torso']} if 'torso' in t else None),
										transitions={'done': 'Plan_to_Starting_Point_Left_Arm'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'trajectories', 'output_value': 'trajectories_torso'})

			# x:227 y:410
			OperatableStateMachine.add('Plan_to_Starting_Point_Right_Leg',
										MoveItMoveGroupPlanState(vel_scaling=0.1),
										transitions={'done': 'Plan_to_Starting_Point_Torso', 'failed': 'Report_Starting_Point_Failure'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'desired_goal': 'trajectories_right_leg', 'plan_to_goal': 'plan_to_goal_right_leg'})

			# x:237 y:531
			OperatableStateMachine.add('Plan_to_Starting_Point_Left_Leg',
										MoveItMoveGroupPlanState(vel_scaling=0.1),
										transitions={'done': 'Plan_to_Starting_Point_Right_Leg', 'failed': 'Report_Starting_Point_Failure'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'desired_goal': 'trajectories_left_leg', 'plan_to_goal': 'plan_to_goal_left_leg'})

			# x:241 y:296
			OperatableStateMachine.add('Plan_to_Starting_Point_Torso',
										MoveItMoveGroupPlanState(vel_scaling=0.1),
										transitions={'done': 'Combine_Plans', 'failed': 'Report_Starting_Point_Failure'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'desired_goal': 'trajectories_torso', 'plan_to_goal': 'plan_to_goal_torso'})


		# x:1090 y:55, x:340 y:59
		_sm_execute_individual_trajectory_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['output_bagfile', 'trajectories', 'experiment_name'])

		with _sm_execute_individual_trajectory_1:
			# x:124 y:257
			OperatableStateMachine.add('Start_Video_Logging',
										VideoLoggingState(command=VideoLoggingState.START, no_video=False, no_bags=True),
										transitions={'done': 'Starting_Point'},
										autonomy={'done': Autonomy.Off},
										remapping={'experiment_name': 'experiment_name', 'description': 'experiment_name'})

			# x:484 y:106
			OperatableStateMachine.add('Stop_Recording_After_Failure',
										StopRecordLogsState(),
										transitions={'stopped': 'Stop_Video_Logging_After_Failure'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:727 y:188
			OperatableStateMachine.add('Wait_for_Settling',
										WaitState(wait_time=1.0),
										transitions={'done': 'Stop_Recording'},
										autonomy={'done': Autonomy.Off})

			# x:482 y:188
			OperatableStateMachine.add('Execute_Trajs_from_Bagfile',
										ExecuteTrajectoryWholeBodyState(controllers=[]),
										transitions={'done': 'Wait_for_Settling', 'failed': 'Stop_Recording_After_Failure'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'trajectories': 'trajectories'})

			# x:903 y:97
			OperatableStateMachine.add('Stop_Video_Logging',
										VideoLoggingState(command=VideoLoggingState.STOP, no_video=False, no_bags=True),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'experiment_name': 'experiment_name', 'description': 'experiment_name'})

			# x:472 y:26
			OperatableStateMachine.add('Stop_Video_Logging_After_Failure',
										VideoLoggingState(command=VideoLoggingState.STOP, no_video=False, no_bags=True),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off},
										remapping={'experiment_name': 'experiment_name', 'description': 'experiment_name'})

			# x:285 y:378
			OperatableStateMachine.add('Starting_Point',
										_sm_starting_point_0,
										transitions={'finished': 'Record_SysID_Test', 'failed': 'Stop_Video_Logging_After_Failure'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'experiment_name': 'experiment_name', 'trajectories': 'trajectories'})

			# x:904 y:187
			OperatableStateMachine.add('Stop_Recording',
										StopRecordLogsState(),
										transitions={'stopped': 'Stop_Video_Logging'},
										autonomy={'stopped': Autonomy.Low},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:494 y:292
			OperatableStateMachine.add('Wait_For_Rosbag_Record',
										WaitState(wait_time=1.0),
										transitions={'done': 'Execute_Trajs_from_Bagfile'},
										autonomy={'done': Autonomy.Low})

			# x:507 y:384
			OperatableStateMachine.add('Record_SysID_Test',
										StartRecordLogsState(topics_to_record=self.topics_to_record),
										transitions={'logging': 'Wait_For_Rosbag_Record'},
										autonomy={'logging': Autonomy.Off},
										remapping={'bagfile_name': 'output_bagfile', 'rosbag_process': 'rosbag_process'})


		# x:1267 y:273, x:406 y:121
		_sm_perform_calibration_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['output_bagfile', 'experiment_name', 'trajectories'], output_keys=['trajectories_command'])

		with _sm_perform_calibration_2:
			# x:136 y:171
			OperatableStateMachine.add('Go_to_Intermediate_Mode',
										ChangeControlModeActionState(target_mode=motion_mode),
										transitions={'changed': 'Execute_Individual_Trajectory', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})

			# x:514 y:314
			OperatableStateMachine.add('Intermediate_Mode_before_exit',
										ChangeControlModeActionState(target_mode=motion_mode),
										transitions={'changed': 'Initial_Mode_before_exit', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:995 y:274
			OperatableStateMachine.add('Initial_Mode_before_exit',
										ChangeControlModeActionState(target_mode=initial_mode),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.High})

			# x:222 y:296
			OperatableStateMachine.add('Execute_Individual_Trajectory',
										_sm_execute_individual_trajectory_1,
										transitions={'finished': 'Intermediate_Mode_before_exit', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'output_bagfile': 'output_bagfile', 'trajectories': 'trajectories', 'experiment_name': 'experiment_name'})


		# x:221 y:562, x:709 y:85
		_sm_update_calibration_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['output_bagfile', 'trajectories', 'calibration_chain_key_local'])

		with _sm_update_calibration_3:
			# x:111 y:94
			OperatableStateMachine.add('Calculate_Calibration',
										CalculateForceTorqueCalibration(calibration_chain=calibration_chain, settlingtime=settlingtime, static_calibration_data=static_calibration_data),
										transitions={'done': 'Ask_Perform_Update', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'bag_filename': 'output_bagfile', 'trajectories_command': 'trajectories', 'ft_calib_data': 'ft_calib_data'})

			# x:420 y:515
			OperatableStateMachine.add('Calibration_Successful',
										LogState(text="Successfully updated calibration offsets.", severity=Logger.REPORT_INFO),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:774 y:277
			OperatableStateMachine.add('Calibration_Failed',
										LogState(text="Failed to apply calibration offsets!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:122 y:202
			OperatableStateMachine.add('Ask_Perform_Update',
										OperatorDecisionState(outcomes=['update', 'no_update'], hint="Do you want to apply the calculated offsets for calibration?", suggestion=None),
										transitions={'update': 'Generate_Keys_Dict', 'no_update': 'finished'},
										autonomy={'update': Autonomy.Full, 'no_update': Autonomy.Full})

			# x:207 y:308
			OperatableStateMachine.add('Generate_Keys_Dict',
										FlexibleCalculationState(calculation=self.create_parameter_keys_dict, input_keys=["input"]),
										transitions={'done': 'Update_Dynamic_Reconfigure'},
										autonomy={'done': Autonomy.Off},
										remapping={'input': 'calibration_chain_key_local', 'output_value': 'parameter_keys_dict'})

			# x:408 y:303
			OperatableStateMachine.add('Update_Dynamic_Reconfigure',
										UpdateDynamicParameterImpedanceControllerState(controller_chain=calibration_chain),
										transitions={'updated': 'Calibration_Successful', 'failed': 'Calibration_Failed'},
										autonomy={'updated': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'parameter_keys': 'parameter_keys_dict', 'parameter_values': 'ft_calib_data'})



		with _state_machine:
			# x:160 y:20
			OperatableStateMachine.add('Execution_Starting',
										LogState(text="Execution has started. Please confirm transition to first state.", severity=Logger.REPORT_HINT),
										transitions={'done': 'Gen_Experiment_Name'},
										autonomy={'done': Autonomy.Full})

			# x:685 y:431
			OperatableStateMachine.add('Update_Calibration',
										_sm_update_calibration_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'output_bagfile': 'output_bagfile', 'trajectories': 'trajectories_command', 'calibration_chain_key_local': 'calibration_chain_user'})

			# x:99 y:533
			OperatableStateMachine.add('Decision_Perform_Experiment',
										OperatorDecisionState(outcomes=["record_data", "use_existing"], hint="Use existing measurement data bag file or record a new one?", suggestion=None),
										transitions={'record_data': 'Perform_Calibration', 'use_existing': 'Update_Calibration'},
										autonomy={'record_data': Autonomy.Low, 'use_existing': Autonomy.Low})

			# x:130 y:185
			OperatableStateMachine.add('Gen_Experiment_Name',
										FlexibleCalculationState(calculation=lambda i: 'FTCalib', input_keys=[]),
										transitions={'done': 'Gen_Output_Bagfile_Name'},
										autonomy={'done': Autonomy.Off},
										remapping={'output_value': 'experiment_name'})

			# x:134 y:267
			OperatableStateMachine.add('Gen_Output_Bagfile_Name',
										CalculationState(calculation=lambda en: os.path.join(bag_folder_out, en) + ".bag"),
										transitions={'done': 'Generate_Textfiles_Dict'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'experiment_name', 'output_value': 'output_bagfile'})

			# x:443 y:252
			OperatableStateMachine.add('Perform_Calibration',
										_sm_perform_calibration_2,
										transitions={'finished': 'Update_Calibration', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'output_bagfile': 'output_bagfile', 'experiment_name': 'experiment_name', 'trajectories': 'trajectories_command', 'trajectories_command': 'trajectories_command'})

			# x:70 y:421
			OperatableStateMachine.add('Load_Traj_From_Txt',
										GenerateTrajectoryFromTxtfileState(chains=calibration_chain, transitiontime=transitiontime, settlingtime=settlingtime),
										transitions={'done': 'Decision_Perform_Experiment', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'txtfilepaths': 'txtfiles_dict', 'trajectories': 'trajectories_command'})

			# x:92 y:350
			OperatableStateMachine.add('Generate_Textfiles_Dict',
										FlexibleCalculationState(calculation=self.create_txtfiles_dict, input_keys=["calibration_chain", "txtfile_left_arm", "txtfile_right_arm"]),
										transitions={'done': 'Load_Traj_From_Txt'},
										autonomy={'done': Autonomy.Off},
										remapping={'calibration_chain': 'calibration_chain_user', 'txtfile_left_arm': 'txtfile_name_left_arm_user', 'txtfile_right_arm': 'txtfile_name_right_arm_user', 'output_value': 'txtfiles_dict'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
		
	def combine_plans(self, plans):
		output_dict = {}
		try:
			left_arm = plans[0]
			right_arm = plans[1]
			left_leg = plans[2]
			right_leg = plans[3]
			torso = plans[4]
			#hacky_dict_combination = lambda p: dict({'left_arm': p[0]}, **{'right_arm': p[1]})
	
			# Logger.loginfo('SystemIdentificationTestsSM:combine_plans: plans="%s"' % str(plans))
	
			if left_arm is not None:
				output_dict['left_arm'] = left_arm.joint_trajectory
	
			if right_arm is not None: 
				output_dict['right_arm'] = right_arm.joint_trajectory
	
			if left_leg is not None:
				output_dict['left_leg'] = left_leg.joint_trajectory
	
			if right_leg is not None: 
				output_dict['right_leg'] = right_leg.joint_trajectory
	
			if torso is not None: 
				output_dict['torso'] = torso.joint_trajectory
		except Exception as e:
			Logger.loginfo('SystemIdentificationTestsSM:combine_plans: Error "%s"' % (str(e)))
		return output_dict
	
	def create_parameter_keys_dict(self, input):
		output_keys_dict = {}
		calibration_chain = input[0]
		for chain in calibration_chain:
			output_keys_dict[chain] = ['mass', 'COM_x', 'COM_y', 'COM_z', 
							'offset_F_x', 'offset_F_y', 'offset_F_z', 
							'offset_T_x', 'offset_T_y', 'offset_T_z']
		return output_keys_dict
	
	def create_txtfiles_dict(self, input):
		output_dict = {}
		calibration_chain = input[0]
		txtfile_left_arm = input[1]
		txtfile_right_arm = input[2]		
		for chain in calibration_chain:
			if chain == "left_arm" and txtfile_left_arm is not None:
				output_dict[chain] = txtfile_left_arm
			if chain == "right_arm" and txtfile_right_arm is not None:
				output_dict[chain] = txtfile_right_arm
		return output_dict	
	# [/MANUAL_FUNC]
