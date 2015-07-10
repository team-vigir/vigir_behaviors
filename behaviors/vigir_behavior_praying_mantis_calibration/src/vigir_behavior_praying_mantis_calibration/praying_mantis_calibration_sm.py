#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_praying_mantis_calibration')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.wait_state import WaitState
from flexbe_atlas_states.execute_trajectory_both_arms_state import ExecuteTrajectoryBothArmsState
from flexbe_atlas_states.current_joint_positions_state import CurrentJointPositionsState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from flexbe_atlas_states.moveit_starting_point_state import MoveitStartingPointState
from flexbe_states.decision_state import DecisionState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_atlas_states.update_joint_calibration_state import UpdateJointCalibrationState
from flexbe_states.log_state import LogState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import os
import time
import pprint

import rospy
from control_msgs.msg import *
from trajectory_msgs.msg import *

from flexbe_core.proxy import ProxyPublisher

from flexbe_behaviors.atlas_definitions import AtlasDefinitions
from flexbe_behaviors.atlas_functions import AtlasFunctions
# [/MANUAL_IMPORT]


'''
Created on Sat Feb 14 2015
@author: Spyros Maniatopoulos
'''
class PrayingMantisCalibrationSM(Behavior):
	'''
	A behavior that moves ATLAS into the "praying mantis" pose upon startup in order to get consistent joint encoder offsets for calibration purposes.
	'''


	def __init__(self):
		super(PrayingMantisCalibrationSM, self).__init__()
		self.name = 'Praying Mantis Calibration'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		self._offset_topic = "/flor/controller/encoder_offsets"
		self._pub =  ProxyPublisher({self._offset_topic: JointTrajectory})
		
		
		self._joint_limits = AtlasDefinitions.arm_joint_limits

		# Define 90 percent positions for both arms (order of joints same as in _joint_names attribute)
		
		# atlas_v5
		# - account for fall protection pads
		# - ignore the lower 3 joints, ie, the electric motor ones
		left_calib_upper  = [-1.4252, -1.4649, +0.1588, +2.2767, +0.1, +0.1, +0.1]
		left_calib_lower  = [+0.5470, +1.2355, +2.9297, +0.1191, -0.1, +1.0, -0.1]
		right_calib_upper = [+1.4914, +1.4296, +0.2118, -2.2899, +0.1, +0.1, +0.1]
		right_calib_lower = [-0.5470, -1.2355, +2.9297, -0.1191, -0.1, -1.0, -0.1]

		# # atlas_v5 (without shoulder pads)
		# left_calib_upper  = [+0.5470, +1.2355, +2.9297, +2.1576, +0.1, +0.1, +0.1]
		# left_calib_lower  = [-1.1869, -1.4296, +0.2118, +0.1191, -1.3, +1.0, -0.1]
		# right_calib_upper = [-0.5470, -1.2355, +2.9297, -2.1576, +0.1, +0.1, +0.1]
		# right_calib_lower = [+1.1869, +1.4296, +0.2118, -0.1191, -1.3, -1.0, -0.1]
		
		self._joint_calib = {'left_arm':  {'upper': left_calib_upper,  'lower': left_calib_lower},
							 'right_arm': {'upper': right_calib_upper, 'lower': right_calib_lower}
		}
		
		self._joint_names = AtlasDefinitions.arm_joint_names

		# [/MANUAL_INIT]

		# Behavior comments:

		# O 47 211 /Perform_Checks/Manipulate_Limits
		# Without this output_key, Check Behavior complains. Because traj_past_limits could in theory be undefined during runtime.



	def create(self):
		initial_mode = "stand"
		motion_mode = "manipulate"
		mantis_mode = "manipulate_limits"
		percent_past_limits = 0.10 # before: 0.075
		# x:788 y:72, x:474 y:133
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.target_limits = 'upper'
		_state_machine.userdata.cycle_counter = 1
		_state_machine.userdata.stand_posture = None # calculated
		_state_machine.userdata.offsets = {'left_arm': dict(), 'right_arm': dict()}

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		self._percent_past_limits = percent_past_limits

		# Create STAND posture trajectory
		_state_machine.userdata.stand_posture = AtlasFunctions.gen_stand_posture_trajectory()

		# [/MANUAL_CREATE]

		# x:222 y:281, x:349 y:167
		_sm_determine_offsets_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['cycle_counter', 'offsets'], output_keys=['offsets'])

		with _sm_determine_offsets_0:
			# x:61 y:53
			OperatableStateMachine.add('Get_Left_Joint_Positions',
										CurrentJointPositionsState(planning_group="l_arm_group"),
										transitions={'retrieved': 'Determine_Closest_Limits_Left', 'failed': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'joint_positions': 'joint_positions'})

			# x:319 y:54
			OperatableStateMachine.add('Determine_Closest_Limits_Left',
										CalculationState(calculation=self.get_closest_limits_left),
										transitions={'done': 'Store_Offsets_Left'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joint_positions', 'output_value': 'joint_limits'})

			# x:598 y:162
			OperatableStateMachine.add('Get_Right_Joint_Positions',
										CurrentJointPositionsState(planning_group="r_arm_group"),
										transitions={'retrieved': 'Determine_Closest_Limits_Right', 'failed': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'joint_positions': 'joint_positions'})

			# x:584 y:275
			OperatableStateMachine.add('Determine_Closest_Limits_Right',
										CalculationState(calculation=self.get_closest_limits_right),
										transitions={'done': 'Store_Offsets_Right'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'joint_positions', 'output_value': 'joint_limits'})

			# x:608 y:54
			OperatableStateMachine.add('Store_Offsets_Left',
										FlexibleCalculationState(calculation=self.store_offsets_left, input_keys=['limits', 'value', 'offsets', 'counter']),
										transitions={'done': 'Get_Right_Joint_Positions'},
										autonomy={'done': Autonomy.Off},
										remapping={'limits': 'joint_limits', 'value': 'joint_positions', 'offsets': 'offsets', 'counter': 'cycle_counter', 'output_value': 'offsets'})

			# x:340 y:274
			OperatableStateMachine.add('Store_Offsets_Right',
										FlexibleCalculationState(calculation=self.store_offsets_right, input_keys=['limits', 'value', 'offsets', 'counter']),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'limits': 'joint_limits', 'value': 'joint_positions', 'offsets': 'offsets', 'counter': 'cycle_counter', 'output_value': 'offsets'})


		# x:528 y:401, x:707 y:282
		_sm_manipulate_limits_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['cycle_counter', 'offsets'], output_keys=['offsets', 'traj_past_limits'])

		with _sm_manipulate_limits_1:
			# x:100 y:156
			OperatableStateMachine.add('Prevent_Runtime_Failure',
										CalculationState(calculation=lambda x: dict()),
										transitions={'done': 'Go_to_MANIPULATE_LIMITS'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'cycle_counter', 'output_value': 'traj_past_limits'})

			# x:387 y:55
			OperatableStateMachine.add('Wait_for_Control_Mode_change',
										WaitState(wait_time=1.0),
										transitions={'done': 'Get_Left_Joint_Positions'},
										autonomy={'done': Autonomy.Low})

			# x:895 y:279
			OperatableStateMachine.add('Gen_Traj_from_90%_to_110%',
										CalculationState(calculation=self.gen_traj_past_limits),
										transitions={'done': 'Go_to_110%_Joint_Limits'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'current_joint_values', 'output_value': 'traj_past_limits'})

			# x:893 y:391
			OperatableStateMachine.add('Go_to_110%_Joint_Limits',
										ExecuteTrajectoryBothArmsState(controllers=['left_arm_traj_controller', 'right_arm_traj_controller']),
										transitions={'done': 'Determine_Offsets', 'failed': 'Determine_Offsets'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'trajectories': 'traj_past_limits'})

			# x:651 y:385
			OperatableStateMachine.add('Determine_Offsets',
										_sm_determine_offsets_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'cycle_counter': 'cycle_counter', 'offsets': 'offsets'})

			# x:648 y:54
			OperatableStateMachine.add('Get_Left_Joint_Positions',
										CurrentJointPositionsState(planning_group="l_arm_group"),
										transitions={'retrieved': 'Get_Right_Joint_Positions', 'failed': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_positions': 'joint_positions_left'})

			# x:904 y:53
			OperatableStateMachine.add('Get_Right_Joint_Positions',
										CurrentJointPositionsState(planning_group="r_arm_group"),
										transitions={'retrieved': 'Generate_Joint_Positions_Struct', 'failed': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_positions': 'joint_positions_right'})

			# x:886 y:168
			OperatableStateMachine.add('Generate_Joint_Positions_Struct',
										FlexibleCalculationState(calculation=lambda ik: {'left_arm': ik[0], 'right_arm': ik[1]}, input_keys=['left', 'right']),
										transitions={'done': 'Gen_Traj_from_90%_to_110%'},
										autonomy={'done': Autonomy.Off},
										remapping={'left': 'joint_positions_left', 'right': 'joint_positions_right', 'output_value': 'current_joint_values'})

			# x:92 y:55
			OperatableStateMachine.add('Go_to_MANIPULATE_LIMITS',
										ChangeControlModeActionState(target_mode=mantis_mode),
										transitions={'changed': 'Wait_for_Control_Mode_change', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})


		# x:574 y:247, x:276 y:549
		_sm_update_calibration_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['offsets'])

		with _sm_update_calibration_2:
			# x:46 y:44
			OperatableStateMachine.add('Process_Offsets',
										CalculationState(calculation=self.process_offsets),
										transitions={'done': 'Print_Offset_Info'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'offsets', 'output_value': 'offsets'})

			# x:227 y:45
			OperatableStateMachine.add('Print_Offset_Info',
										CalculationState(calculation=self.print_offset_info),
										transitions={'done': 'Publish_Offsets'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'offsets', 'output_value': 'none'})

			# x:390 y:158
			OperatableStateMachine.add('Ask_Perform_Update',
										OperatorDecisionState(outcomes=['update', 'no_update'], hint="Do you want to apply the calculated offsets for calibration?", suggestion=None),
										transitions={'update': 'Convert_Offset_Data', 'no_update': 'finished'},
										autonomy={'update': Autonomy.Full, 'no_update': Autonomy.Full})

			# x:232 y:337
			OperatableStateMachine.add('Update_Calibration',
										UpdateJointCalibrationState(joint_names=self._joint_names['left_arm'][0:4] + self._joint_names['right_arm'][0:4]),
										transitions={'updated': 'Calibration_Successful', 'failed': 'Calibration_Failed'},
										autonomy={'updated': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_offsets': 'offset_list'})

			# x:241 y:242
			OperatableStateMachine.add('Convert_Offset_Data',
										CalculationState(calculation=lambda o: o['left_arm']['avg'] + o['right_arm']['avg']),
										transitions={'done': 'Update_Calibration'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'offsets', 'output_value': 'offset_list'})

			# x:522 y:337
			OperatableStateMachine.add('Calibration_Successful',
										LogState(text="Successfully updated calibration offsets.", severity=Logger.REPORT_INFO),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:246 y:445
			OperatableStateMachine.add('Calibration_Failed',
										LogState(text="Failed to apply calibration offsets!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:399 y:44
			OperatableStateMachine.add('Publish_Offsets',
										CalculationState(calculation=self.publish_offsets),
										transitions={'done': 'Ask_Perform_Update'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'offsets', 'output_value': 'none'})


		# x:978 y:197, x:394 y:80
		_sm_perform_checks_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['cycle_counter', 'target_limits', 'offsets'], output_keys=['cycle_counter', 'offsets'])

		with _sm_perform_checks_3:
			# x:105 y:74
			OperatableStateMachine.add('Go_to_Intermediate_Mode',
										ChangeControlModeActionState(target_mode=motion_mode),
										transitions={'changed': 'Gen_Traj_to_90%_Limits', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:653 y:274
			OperatableStateMachine.add('Manipulate_Limits',
										_sm_manipulate_limits_1,
										transitions={'finished': 'Gen_Traj_back_to_90%_Limits', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'cycle_counter': 'cycle_counter', 'offsets': 'offsets', 'traj_past_limits': 'traj_past_limits'})

			# x:903 y:78
			OperatableStateMachine.add('Increment_Cycle_counter',
										CalculationState(calculation=lambda counter: counter + 1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'cycle_counter', 'output_value': 'cycle_counter'})

			# x:344 y:277
			OperatableStateMachine.add('Move_to_90%_Joint_Limits',
										MoveitStartingPointState(vel_scaling=0.3),
										transitions={'reached': 'Manipulate_Limits', 'failed': 'Move_to_90%_Joint_Limits'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'trajectories': 'trajectories_90'})

			# x:114 y:276
			OperatableStateMachine.add('Gen_Traj_to_90%_Limits',
										CalculationState(calculation=self.gen_traj_pre_limits),
										transitions={'done': 'Move_to_90%_Joint_Limits'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'target_limits', 'output_value': 'trajectories_90'})

			# x:636 y:78
			OperatableStateMachine.add('Go_back_to_90%_Joint_Limits',
										ExecuteTrajectoryBothArmsState(controllers=['left_arm_traj_controller', 'right_arm_traj_controller']),
										transitions={'done': 'Increment_Cycle_counter', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'trajectories': 'traj_back_to_90'})

			# x:636 y:172
			OperatableStateMachine.add('Gen_Traj_back_to_90%_Limits',
										FlexibleCalculationState(calculation=self.gen_traj_back_from_limits, input_keys=['trajectories_90', 'traj_past_limits']),
										transitions={'done': 'Go_back_to_90%_Joint_Limits'},
										autonomy={'done': Autonomy.Off},
										remapping={'trajectories_90': 'trajectories_90', 'traj_past_limits': 'traj_past_limits', 'output_value': 'traj_back_to_90'})



		with _state_machine:
			# x:110 y:52
			OperatableStateMachine.add('Initial_Control_Mode',
										ChangeControlModeActionState(target_mode=initial_mode),
										transitions={'changed': 'Perform_Checks', 'failed': 'failed'},
										autonomy={'changed': Autonomy.High, 'failed': Autonomy.High})

			# x:712 y:317
			OperatableStateMachine.add('Initial_Mode_before_exit',
										ChangeControlModeActionState(target_mode=initial_mode),
										transitions={'changed': 'Update_Calibration', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:122 y:302
			OperatableStateMachine.add('Perform_Checks',
										_sm_perform_checks_3,
										transitions={'finished': 'Are_We_Done_Yet?', 'failed': 'Intermediate_Mode_before_exit'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'cycle_counter': 'cycle_counter', 'target_limits': 'target_limits', 'offsets': 'offsets'})

			# x:126 y:505
			OperatableStateMachine.add('Are_We_Done_Yet?',
										DecisionState(outcomes=["done", "more"], conditions=lambda counter: "done" if counter >= 2 else "more"),
										transitions={'done': 'Intermediate_Mode_before_exit', 'more': 'Setup_next_Cycle'},
										autonomy={'done': Autonomy.Low, 'more': Autonomy.High},
										remapping={'input_value': 'cycle_counter'})

			# x:15 y:404
			OperatableStateMachine.add('Setup_next_Cycle',
										CalculationState(calculation=lambda lim: 'lower' if lim == 'upper' else 'upper'),
										transitions={'done': 'Perform_Checks'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'target_limits', 'output_value': 'target_limits'})

			# x:725 y:186
			OperatableStateMachine.add('Update_Calibration',
										_sm_update_calibration_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'offsets': 'offsets'})

			# x:726 y:427
			OperatableStateMachine.add('Move_to_Stand_Posture',
										MoveitStartingPointState(vel_scaling=0.3),
										transitions={'reached': 'Initial_Mode_before_exit', 'failed': 'Move_to_Stand_Posture'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'trajectories': 'stand_posture'})

			# x:412 y:427
			OperatableStateMachine.add('Intermediate_Mode_before_exit',
										ChangeControlModeActionState(target_mode=motion_mode),
										transitions={'changed': 'Move_to_Stand_Posture', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	def gen_traj_pre_limits(self, limits_side):
		"""Create trajectories for going to 90 percent of joint limits (either upper or lower limits)"""

		joint_config = {'left_arm':  self._joint_calib['left_arm'][limits_side],
						'right_arm': self._joint_calib['right_arm'][limits_side]
		}

		return AtlasFunctions.gen_arm_trajectory_from_joint_configuration(joint_config)

	def _get_closest_limits(self, side, current_values):
		"""
		Selects the closest limit with respect to the current value (upper or lower bound).
		"""
		limits = self._joint_limits[side]
		closest_limit = list()

		for i in range(len(current_values)):
			near_limit = 'upper' if abs(limits['upper'][i] - current_values[i]) < abs(limits['lower'][i] - current_values[i]) else 'lower'
			closest_limit.append(limits[near_limit][i])

		rospy.loginfo("Limit joint positions: %s" % str(closest_limit))
		rospy.loginfo("Current joint positions: %s" % str(current_values))

		return closest_limit

	def get_closest_limits_left(self, current_values):
		return self._get_closest_limits('left_arm', current_values)
	def get_closest_limits_right(self, current_values):
		return self._get_closest_limits('right_arm', current_values)


	def gen_traj_past_limits(self, current_joint_values):
		"""
		Given all joint limits, generate a trajectory that takes the joints to 110%% percent past limits.

		atlas_v5 update: Do not push the lower 3 joints (electric ones) path the limits.
		"""
		
		result = dict()

		for arm in ['left_arm', 'right_arm']:
			current_values = current_joint_values[arm]
			arm_limits = self._get_closest_limits(arm, current_values)
			arm_target = list()
			arm_effort = list()
			percentage = self._percent_past_limits

			# Push the upper 4 joints against the limits
			for i in range(0,4):
				near_limit = 'upper' if self._joint_limits[arm]['upper'][i] == arm_limits[i] else 'lower'
				limit_range = self._joint_limits[arm]['upper'][i] - self._joint_limits[arm]['lower'][i]
				offset_sign = 1 if near_limit is 'upper' else -1
				arm_target.append(arm_limits[i] + offset_sign * percentage * limit_range)
				arm_effort.append(float(offset_sign))

			# "Ignore" the lower 3 joints (electric motor ones)
			for i in range(4,7):
				arm_target.append(current_values[i])
				arm_effort.append(0.0)	# Zero effort stands for not applying additional force

			trajectory = JointTrajectory()
			trajectory.joint_names = self._joint_names[arm]

			point = JointTrajectoryPoint()
			point.positions = arm_target
			point.velocities = [0.0] * len(arm_target) # David's controller expects zero velocities
			point.effort = arm_effort
			point.time_from_start = rospy.Duration.from_sec(2.5)
			trajectory.points.append(point)

			# rospy.loginfo("110%% joint positions for %s arm: %s" % (arm, str(arm_target[0:4]))) # Only report the relevant joints

			result[arm] = trajectory

		return result

	def gen_traj_back_from_limits(self, input_keys):
		"""The resulting trajectory points are the same as for going to 90%% of limits, but with the efforts set for David's controllers."""
		
		traj_pre_limits  = input_keys[0]
		traj_past_limits = input_keys[1]

		traj_back_to_90 = dict()

		for arm in ['left_arm', 'right_arm']:

			trajectory = traj_pre_limits[arm]	# Start with 90% of joint limits as the trajectory points

			trajectory.points[0].effort = traj_past_limits[arm].points[0].effort # Set efforts as per David's controllers
			trajectory.points[0].time_from_start = rospy.Duration.from_sec(1.0)

			# David's controller expects zero velocities
			trajectory.points[0].velocities = [0.0] * len(trajectory.points[0].positions)

			traj_back_to_90[arm] = trajectory

		return traj_back_to_90

	def store_offsets(self, side, input_keys):
		limits = input_keys[0][0:4]	# Ignore the lower 3 joints
		values = input_keys[1][0:4]	# 	--//--  	--//--	
		offsets = input_keys[2]
		counter = input_keys[3]

		offsets[side][counter] = [limit - value for limit, value in zip(limits, values)]

		msg = JointTrajectory()
		msg.joint_names = self._joint_names[side][0:4]	# Ignore the lower 3 joints

		point = JointTrajectoryPoint()
		point.positions = values
		point.velocities = offsets[side][counter]

		msg.points.append(point)

		self._pub.publish(self._offset_topic, msg)
		Logger.loginfo("Publishing %s arm offsets to %s" % (side, self._offset_topic))

		return offsets

	def publish_offsets(self, offsets, arms = ['left_arm', 'right_arm'], current_values = []):

		for side in arms:

			msg = JointTrajectory()
			msg.joint_names = self._joint_names[side]

			point = JointTrajectoryPoint()
			point.positions = current_values
			point.velocities = offsets[side]['avg']

			msg.points.append(point)

			self._pub.publish(self._offset_topic, msg)
			Logger.loginfo("Publishing %s arm offsets to %s" % (side, self._offset_topic))

	def store_offsets_left(self, input_keys):
		return self.store_offsets('left_arm', input_keys)
	
	def store_offsets_right(self, input_keys):
		return self.store_offsets('right_arm', input_keys)

	def process_offsets(self, offsets):

		for side in ['left_arm', 'right_arm']:
			# transposes list of lists from iteration,joint to joint,iteration
			iteration_values = map(list, zip(*offsets[side].values()))
			# Calculate the average offset and the deviation from the average
			offsets[side]['avg'] = [sum(joint_entries)/float(len(joint_entries)) for joint_entries in iteration_values]
			offsets[side]['diff'] = [max(map(lambda x: abs(x-avg),joint_entries)) for joint_entries,avg in zip(iteration_values, offsets[side]['avg'])]

		return offsets


	def print_offset_info(self, offsets):
		sides = ['left_arm', 'right_arm']
		for side in sides:
			Logger.loginfo("Joint order (%s): %s" % (side, str(self._joint_names[side][0:4])))
			rounded_offsets = [round(offset, 3) for offset in offsets[side]['avg']] # round due to comms_bridge
			Logger.loginfo("Offsets (%s): %s"  % (side, str(rounded_offsets)))
			# Logger.loginfo("Max deviation from average (%s): %s" % (side, str(offsets[side]['diff'])))

		pprint.pprint(offsets) # Pretty print to the "onboard" terminal
	# [/MANUAL_FUNC]
