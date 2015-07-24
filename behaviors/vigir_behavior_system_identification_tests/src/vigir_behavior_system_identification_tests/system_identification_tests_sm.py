#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_system_identification_tests')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.stop_record_logs_state import StopRecordLogsState
from flexbe_states.wait_state import WaitState
from flexbe_states.start_record_logs_state import StartRecordLogsState
from vigir_flexbe_states.execute_trajectory_whole_body_state import ExecuteTrajectoryWholeBodyState
from vigir_flexbe_states.load_trajectory_from_bagfile_state import LoadTrajectoryFromBagfileState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from vigir_flexbe_states.video_logging_state import VideoLoggingState
from vigir_flexbe_states.moveit_move_group_plan_state import MoveItMoveGroupPlanState
from flexbe_states.decision_state import DecisionState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import os
import time
import glob

import rospy
from control_msgs.msg import *
from trajectory_msgs.msg import *

from vigir_flexbe_behaviors.atlas_functions import AtlasFunctions
# [/MANUAL_IMPORT]


'''
Created on Fri Jan 09 2015
@author: Spyros Maniatopoulos
'''
class SystemIdentificationTestsSM(Behavior):
	'''
	A behavior for executing predefined joint-space trajectories for system identification purposes
	'''


	def __init__(self):
		super(SystemIdentificationTestsSM, self).__init__()
		self.name = 'System Identification Tests'

		# parameters of this behavior
		self.add_parameter('topics_to_record', '')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT] 

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		bag_folder_in = "~/sysid_tests"
		bag_folder_out = "" # optional (if not provided, an /out folder will be created in the bag_folder_in directory
		prefix = "" # optional (applies to bag and video files)
		input_bagfiles = [] # calculated (leave empty)
		output_bagfiles = [] # calculated (leave empty)
		desired_num_tests = 1 # num of tests per trajectory
		wait_time = 3.0 # before execution (for rosbag record)
		settling_time = 2.0 # after execution
		mode_for_tests = "dance" # "manipulate", "system_id", etc.
		desired_controllers = ["left_arm_traj_controller","right_arm_traj_controller", "left_leg_traj_controller","right_leg_traj_controller","torso_traj_controller"]
		# x:880 y:104, x:660 y:14
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.traj_index = 0
		_state_machine.userdata.tests_counter = 0
		_state_machine.userdata.none = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# Get folder containing input bagfiles and create folder for output

		bag_folder_in = os.path.expanduser(bag_folder_in)
		if not os.path.exists(bag_folder_in):
			Logger.logwarn('Path to input bag folder does not exist (%s)' % bag_folder_in)
		
		if not bag_folder_out:
			bag_folder_out = os.path.join(bag_folder_in, 'out')
			if not os.path.exists(bag_folder_out):
				os.makedirs(bag_folder_out)
		else:
			# The user has provided a directory for output bagfiles
			bag_folder_out = os.path.expanduser(bag_folder_out)
			if not os.path.exists(bag_folder_out):
				os.makedirs(bag_folder_out)

		# Initialize lists again, in case the user did alter them
		input_bagfiles  = []
		output_bagfiles = []

		# Get all input bagfile names from bag folder and also name the corresponding output bagfiles
		os.chdir(bag_folder_in)
		for bagfile in sorted(glob.glob("*.bag")):
			input_bagfiles.append(os.path.join(bag_folder_in, bagfile))
			bare_name = bagfile.split(".")[0]
			output_name = bare_name + "_" + time.strftime("%Y-%m-%d-%H_%M") + "_" # file name will be completed by flexible calculation state
			output_bagfiles.append(output_name)
		
		Logger.loginfo('Found %d input bag files in %s' % (len(input_bagfiles), bag_folder_in))

		# Create STAND posture trajectories
		_state_machine.userdata.stand_posture = AtlasFunctions.gen_stand_posture_trajectory()

		# [/MANUAL_CREATE]

		# x:836 y:51, x:858 y:296
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
										WaitState(wait_time=wait_time),
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
										ExecuteTrajectoryWholeBodyState(controllers=desired_controllers),
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


		# x:1118 y:103, x:348 y:32
		_sm_execute_individual_trajectory_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['traj_index', 'tests_counter'])

		with _sm_execute_individual_trajectory_1:
			# x:79 y:28
			OperatableStateMachine.add('Get_Input_Bagfile',
										CalculationState(calculation=lambda idx: input_bagfiles[idx]),
										transitions={'done': 'Gen_Experiment_Name'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'traj_index', 'output_value': 'input_bagfile'})

			# x:904 y:187
			OperatableStateMachine.add('Stop_Recording',
										StopRecordLogsState(),
										transitions={'stopped': 'Stop_Video_Logging'},
										autonomy={'stopped': Autonomy.Low},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:494 y:292
			OperatableStateMachine.add('Wait_For_Rosbag_Record',
										WaitState(wait_time=wait_time),
										transitions={'done': 'Execute_Trajs_from_Bagfile'},
										autonomy={'done': Autonomy.Low})

			# x:507 y:384
			OperatableStateMachine.add('Record_SysID_Test',
										StartRecordLogsState(topics_to_record=self.topics_to_record),
										transitions={'logging': 'Wait_For_Rosbag_Record'},
										autonomy={'logging': Autonomy.Off},
										remapping={'bagfile_name': 'output_bagfile', 'rosbag_process': 'rosbag_process'})

			# x:484 y:106
			OperatableStateMachine.add('Stop_Recording_After_Failure',
										StopRecordLogsState(),
										transitions={'stopped': 'Stop_Video_Logging_After_Failure'},
										autonomy={'stopped': Autonomy.Off},
										remapping={'rosbag_process': 'rosbag_process'})

			# x:727 y:188
			OperatableStateMachine.add('Wait_for_Settling',
										WaitState(wait_time=settling_time),
										transitions={'done': 'Stop_Recording'},
										autonomy={'done': Autonomy.Off})

			# x:482 y:188
			OperatableStateMachine.add('Execute_Trajs_from_Bagfile',
										ExecuteTrajectoryWholeBodyState(controllers=desired_controllers),
										transitions={'done': 'Wait_for_Settling', 'failed': 'Stop_Recording_After_Failure'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'trajectories': 'trajectories'})

			# x:48 y:283
			OperatableStateMachine.add('Load_Trajs_from_Bagfile',
										LoadTrajectoryFromBagfileState(),
										transitions={'done': 'Start_Video_Logging', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'bagfile_name': 'input_bagfile', 'trajectories': 'trajectories'})

			# x:65 y:113
			OperatableStateMachine.add('Gen_Experiment_Name',
										FlexibleCalculationState(calculation=lambda i: prefix + output_bagfiles[i[0]] + str(i[1] + 1), input_keys=["idx", "counter"]),
										transitions={'done': 'Gen_Output_Bagfile_Name'},
										autonomy={'done': Autonomy.Off},
										remapping={'idx': 'traj_index', 'counter': 'tests_counter', 'output_value': 'experiment_name'})

			# x:56 y:192
			OperatableStateMachine.add('Gen_Output_Bagfile_Name',
										CalculationState(calculation=lambda en: os.path.join(bag_folder_out, en) + ".bag"),
										transitions={'done': 'Load_Trajs_from_Bagfile'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'experiment_name', 'output_value': 'output_bagfile'})

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

			# x:71 y:384
			OperatableStateMachine.add('Start_Video_Logging',
										VideoLoggingState(command=VideoLoggingState.START, no_video=False, no_bags=True),
										transitions={'done': 'Starting_Point'},
										autonomy={'done': Autonomy.Off},
										remapping={'experiment_name': 'experiment_name', 'description': 'experiment_name'})


		# x:475 y:405
		_sm_execute_sysid_trajectories_2 = OperatableStateMachine(outcomes=['finished'], input_keys=['traj_index', 'tests_counter'], output_keys=['traj_index'])

		with _sm_execute_sysid_trajectories_2:
			# x:367 y:24
			OperatableStateMachine.add('Execute_Individual_Trajectory',
										_sm_execute_individual_trajectory_1,
										transitions={'finished': 'Increment_Tests_per_Traj_counter', 'failed': 'Wait_before_Fail'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'traj_index': 'traj_index', 'tests_counter': 'tests_counter'})

			# x:714 y:27
			OperatableStateMachine.add('Increment_Tests_per_Traj_counter',
										CalculationState(calculation=lambda counter: counter + 1),
										transitions={'done': 'More_Tests_for_this_Traj?'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'tests_counter', 'output_value': 'tests_counter'})

			# x:738 y:207
			OperatableStateMachine.add('More_Tests_for_this_Traj?',
										DecisionState(outcomes=['yes', 'no'], conditions=lambda counter: 'no' if counter >= desired_num_tests else 'yes'),
										transitions={'yes': 'Execute_Individual_Trajectory', 'no': 'Reset_Tests_counter'},
										autonomy={'yes': Autonomy.Low, 'no': Autonomy.Low},
										remapping={'input_value': 'tests_counter'})

			# x:752 y:298
			OperatableStateMachine.add('Reset_Tests_counter',
										CalculationState(calculation=lambda counter: 0),
										transitions={'done': 'Increment_Traj_Index'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'tests_counter', 'output_value': 'tests_counter'})

			# x:752 y:399
			OperatableStateMachine.add('Increment_Traj_Index',
										CalculationState(calculation=lambda idx: idx + 1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'traj_index', 'output_value': 'traj_index'})

			# x:405 y:141
			OperatableStateMachine.add('Wait_before_Fail',
										WaitState(wait_time=2.0),
										transitions={'done': 'Increment_Tests_per_Traj_counter'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:122 y:26
			OperatableStateMachine.add('Starting_Execution',
										LogState(text="Execution is starting. Confirm first transition!", severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_to_Desired_Mode'},
										autonomy={'done': Autonomy.High})

			# x:328 y:298
			OperatableStateMachine.add('Execute SysID Trajectories',
										_sm_execute_sysid_trajectories_2,
										transitions={'finished': 'More_Trajectories?'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'traj_index': 'traj_index', 'tests_counter': 'tests_counter'})

			# x:129 y:215
			OperatableStateMachine.add('More_Trajectories?',
										DecisionState(outcomes=['yes', 'no'], conditions=lambda idx: 'no' if idx >= len(input_bagfiles) else 'yes'),
										transitions={'yes': 'Notify_Next_Trajectory', 'no': 'Move_to_Stand_Posture'},
										autonomy={'yes': Autonomy.Low, 'no': Autonomy.High},
										remapping={'input_value': 'traj_index'})

			# x:592 y:103
			OperatableStateMachine.add('Go_to_FREEZE_before_Exit',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.FREEZE),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Low})

			# x:584 y:214
			OperatableStateMachine.add('Failed_To_Go_To_Stand_Posture',
										LogState(text="Failed to go to stand posture", severity=Logger.REPORT_WARN),
										transitions={'done': 'Go_to_FREEZE_before_Exit'},
										autonomy={'done': Autonomy.Off})

			# x:121 y:393
			OperatableStateMachine.add('Notify_Next_Trajectory',
										LogState(text="Continuing with next trajectory.", severity=Logger.REPORT_INFO),
										transitions={'done': 'Execute SysID Trajectories'},
										autonomy={'done': Autonomy.Off})

			# x:112 y:102
			OperatableStateMachine.add('Go_to_Desired_Mode',
										ChangeControlModeActionState(target_mode=mode_for_tests),
										transitions={'changed': 'More_Trajectories?', 'failed': 'Go_to_FREEZE_before_Exit'},
										autonomy={'changed': Autonomy.High, 'failed': Autonomy.High})

			# x:329 y:158
			OperatableStateMachine.add('Move_to_Stand_Posture',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.3, ignore_collisions=False, link_paddings={}),
										transitions={'done': 'Go_to_FREEZE_before_Exit', 'failed': 'Failed_To_Go_To_Stand_Posture'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.High},
										remapping={'side': 'none'})


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
				Logger.loginfo('SystemIdentificationTestsSM:combine_plans: Found left arm joint trajectory')
				output_dict['left_arm'] = left_arm.joint_trajectory
	
			if right_arm is not None: 
				Logger.loginfo('SystemIdentificationTestsSM:combine_plans: Found right arm joint trajectory')
				output_dict['right_arm'] = right_arm.joint_trajectory
	
			if left_leg is not None:
				Logger.loginfo('SystemIdentificationTestsSM:combine_plans: Found left leg joint trajectory')
				output_dict['left_leg'] = left_leg.joint_trajectory
	
			if right_leg is not None: 
				Logger.loginfo('SystemIdentificationTestsSM:combine_plans: Found right leg joint trajectory')
				output_dict['right_leg'] = right_leg.joint_trajectory
	
			if torso is not None: 
				Logger.loginfo('SystemIdentificationTestsSM:combine_plans: Found torso joint trajectory')
				output_dict['torso'] = torso.joint_trajectory
		except Exception as e:
			Logger.loginfo('SystemIdentificationTestsSM:combine_plans: Error "%s"' % (str(e)))
		return output_dict

	# [/MANUAL_FUNC]
