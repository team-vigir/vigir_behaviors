#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_steering_calibration')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from flexbe_states.input_state import InputState
from vigir_flexbe_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_flexbe_states.plan_affordance_state import PlanAffordanceState
from flexbe_states.decision_state import DecisionState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from vigir_flexbe_states.query_joint_positions_state import QueryJointPositionsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import math

import rospy
from rospkg import RosPack

import yaml
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
		self.add_parameter('angle_increment', 30)
		self.add_parameter('save_path', '/save123.yaml')
		self.add_parameter('hand_side', 'left')
		self.add_parameter('save_ros_package', 'thor_mang_driving_controller')
		self.add_parameter('move_to_poses', False)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		self.key_positions = {};
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		# x:1234 y:456, x:77 y:585
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.reference_point = None
		_state_machine.userdata.current_target_angle_deg = 0
		_state_machine.userdata.dummy = None
		_state_machine.userdata.save_joints = ['r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', 'r_wrist_yaw1', 'r_wrist_roll', 'r_wrist_yaw2']
		_state_machine.userdata.move_to_poses = self.move_to_poses

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:1136 y:74, x:130 y:480
		_sm_save_target_arm_pose_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['joint_trajectory', 'hand_side', 'move_to_poses'])

		with _sm_save_target_arm_pose_0:
			# x:146 y:184
			OperatableStateMachine.add('Decide_Move_Arm',
										DecisionState(outcomes=['move_arm', 'do_not_move'], conditions=self.decide_move_arm),
										transitions={'move_arm': 'Move_Arm_To_Pose', 'do_not_move': 'Save_Arm_Pose_From_Trajectory'},
										autonomy={'move_arm': Autonomy.Low, 'do_not_move': Autonomy.Low},
										remapping={'input_value': 'move_to_poses'})

			# x:330 y:232
			OperatableStateMachine.add('Move_Arm_To_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Get_Current_Joint_Values', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:875 y:223
			OperatableStateMachine.add('Save_Arm_Pose_From_Joints',
										CalculationState(calculation=self.save_arm_pose_from_joints),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'joint_config', 'output_value': 'success'})

			# x:330 y:45
			OperatableStateMachine.add('Save_Arm_Pose_From_Trajectory',
										CalculationState(calculation=self.save_arm_pose_from_trajectory),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'joint_trajectory', 'output_value': 'success'})

			# x:622 y:225
			OperatableStateMachine.add('Get_Current_Joint_Values',
										QueryJointPositionsState(side=self.hand_side, controller='traj_controller'),
										transitions={'retrieved': 'Save_Arm_Pose_From_Joints', 'failed': 'failed'},
										autonomy={'retrieved': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joint_config': 'joint_config'})


		# x:1319 y:112, x:44 y:619
		_sm_calculate_key_frames_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['wheel_affordance', 'hand_side', 'reference_point', 'move_to_poses'])

		with _sm_calculate_key_frames_1:
			# x:334 y:65
			OperatableStateMachine.add('Plan_Wheel_Rotation',
										PlanAffordanceState(vel_scaling=0.1, planner_id="drake", drake_sample_rate=4.0, drake_orientation_type=1, drake_link_axis=[0,0,1]),
										transitions={'done': 'Save_Target_Arm_Pose', 'incomplete': 'Decide_Continue', 'failed': 'Decide_Continue'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'affordance': 'wheel_affordance', 'hand': 'hand_side', 'reference_point': 'reference_point', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:998 y:85
			OperatableStateMachine.add('Continue_Rotation',
										DecisionState(outcomes=['continue_rotation', 'finished'], conditions=self.check_continue_rotation),
										transitions={'continue_rotation': 'Plan_Wheel_Rotation', 'finished': 'finished'},
										autonomy={'continue_rotation': Autonomy.Low, 'finished': Autonomy.Low},
										remapping={'input_value': 'wheel_affordance'})

			# x:921 y:470
			OperatableStateMachine.add('Calculate_New_Target_Angle',
										CalculationState(calculation=self.calc_rotation_angle),
										transitions={'done': 'Continue_Rotation'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'wheel_affordance', 'output_value': 'wheel_affordance'})

			# x:327 y:459
			OperatableStateMachine.add('Decide_Continue',
										OperatorDecisionState(outcomes=['continue', 'retry', 'abort'], hint=None, suggestion=None),
										transitions={'continue': 'Calculate_New_Target_Angle', 'retry': 'Plan_Wheel_Rotation', 'abort': 'failed'},
										autonomy={'continue': Autonomy.High, 'retry': Autonomy.High, 'abort': Autonomy.High})

			# x:670 y:190
			OperatableStateMachine.add('Save_Target_Arm_Pose',
										_sm_save_target_arm_pose_0,
										transitions={'finished': 'Calculate_New_Target_Angle', 'failed': 'Decide_Continue'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'joint_trajectory': 'joint_trajectory', 'hand_side': 'hand_side', 'move_to_poses': 'move_to_poses'})


		# x:936 y:246, x:87 y:494
		_sm_get_template_affordance_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side'], output_keys=['wheel_affordance'])

		with _sm_get_template_affordance_2:
			# x:71 y:85
			OperatableStateMachine.add('Request_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the STEERING_WHEEL template."),
										transitions={'received': 'Get_Template_Affordance', 'aborted': 'failed', 'no_connection': 'Log_No_Connection', 'data_error': 'Log_Data_Error'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'wheel_template_id'})

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

			# x:600 y:230
			OperatableStateMachine.add('Get_Template_Affordance',
										GetTemplateAffordanceState(identifier='turn_right'),
										transitions={'done': 'finished', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low, 'not_available': Autonomy.Low},
										remapping={'template_id': 'wheel_template_id', 'hand_side': 'hand_side', 'affordance': 'wheel_affordance'})



		with _state_machine:
			# x:61 y:59
			OperatableStateMachine.add('Notify_Execution_Started',
										LogState(text="Execution has started. Consider making modifications if this is a re-run.", severity=Logger.REPORT_HINT),
										transitions={'done': 'Get_Template_Affordance'},
										autonomy={'done': Autonomy.High})

			# x:27 y:165
			OperatableStateMachine.add('Get_Template_Affordance',
										_sm_get_template_affordance_2,
										transitions={'finished': 'Init_Affordance', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'wheel_affordance': 'wheel_affordance'})

			# x:1080 y:318
			OperatableStateMachine.add('Notify_Behavior_Exit',
										LogState(text='The behavior will now exit. Last chance to intervene!', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})

			# x:325 y:170
			OperatableStateMachine.add('Init_Affordance',
										CalculationState(calculation=self.init_affordance),
										transitions={'done': 'Calculate_Key_Frames'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'wheel_affordance', 'output_value': 'wheel_affordance'})

			# x:732 y:320
			OperatableStateMachine.add('Decide_Save',
										OperatorDecisionState(outcomes=['save', 'no_save'], hint=None, suggestion=None),
										transitions={'save': 'Save_Key_Positions', 'no_save': 'Notify_Behavior_Exit'},
										autonomy={'save': Autonomy.Full, 'no_save': Autonomy.Full})

			# x:925 y:424
			OperatableStateMachine.add('Save_Key_Positions',
										CalculationState(calculation=self.save_key_positions_to_disc),
										transitions={'done': 'Notify_Behavior_Exit'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'dummy', 'output_value': 'output_value'})

			# x:471 y:319
			OperatableStateMachine.add('Calculate_Key_Frames',
										_sm_calculate_key_frames_1,
										transitions={'finished': 'Decide_Save', 'failed': 'Decide_Save'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'wheel_affordance': 'wheel_affordance', 'hand_side': 'hand_side', 'reference_point': 'reference_point', 'move_to_poses': 'move_to_poses'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def init_affordance(self, affordance):
		'''Initializes affordance with a 0 degree displacement angle'''
		
		affordance.displacement = 0.0
		Logger.loginfo("Initialized affordance with rotation of %f degrees." % math.degrees(affordance.displacement))
		return affordance
	
	def calc_rotation_angle(self, affordance):
		'''Calculates the angle for the next planning step'''
		if ( self._state_machine.userdata.move_to_poses ):
			new_rotation_angle = math.radians(self.angle_increment)
		else:
			new_rotation_angle = affordance.displacement + math.radians(self.angle_increment);	
		
		self._state_machine.userdata.current_target_angle_deg = self._state_machine.userdata.current_target_angle_deg + self.angle_increment
		affordance.displacement = new_rotation_angle
		
		Logger.loginfo("Next rotation step: %f degrees (%f increment from current position)" % (self._state_machine.userdata.current_target_angle_deg, math.degrees(new_rotation_angle)))
		
		return affordance
	
	def check_continue_rotation(self, affordance):	
		'''Check if we have made a full circle'''
		if ( affordance.displacement > 2*math.pi ):
			return 'finished'
		else:
			return 'continue_rotation'
		
	def decide_move_arm(self, move_arm):
		if ( move_arm ):
			return 'move_arm'
		else:
			return 'do_not_move'
		
	def save_arm_pose_from_trajectory(self, joint_trajectory):
		target_angle = self._state_machine.userdata.current_target_angle_deg
		
		last_idx = len(joint_trajectory.points)-1
		joint_names = joint_trajectory.joint_names
		joint_values = joint_trajectory.points[last_idx].positions
		
		self.add_key_position(target_angle, self.get_target_position_from_joints(joint_names, joint_values))
		Logger.loginfo("Saving joint angle for %d degrees rotation" % target_angle)
		return True
	
	def save_arm_pose_from_joints(self, joint_config):
		target_angle = self._state_machine.userdata.current_target_angle_deg
		joint_names = joint_config['joint_names']
		joint_values = joint_config['joint_values']
		
		self.add_key_position(target_angle, self.get_target_position_from_joints(joint_names, joint_values))
		Logger.loginfo("Saving joint angle for %d degrees rotation" % target_angle)
		return True
	
	def add_key_position(self, angle, joint_positions):
		if joint_positions is not None:
			self.key_positions[angle] = joint_positions
	
	def save_key_positions_to_disc(self, dummy):
		rp = RosPack()
		save_path = rp.get_path(self.save_ros_package) + self.save_path
		
		Logger.loginfo("Saving key positions to %s" % save_path)

		yaml_dict = dict()
		yaml_dict.update({'joints': self._state_machine.userdata.save_joints})
		yaml_dict.update({'angles': [int(angle) for angle in self.key_positions.iterkeys()]})
		yaml_dict.update({'angle_' + str(angle).split('.')[0]: self.key_positions[angle] for angle in self.key_positions})
		with open(save_path, 'w+') as outfile:
			outfile.write(yaml.dump(yaml_dict, default_flow_style=False))

	def get_target_position_from_joints(self, joint_names, joint_values): 
		try:
			joint_ids = [joint_names.index(joint_name) for joint_name in self._state_machine.userdata.save_joints]
		except ValueError as e:
			print 'Some joint was not found in received joint state:\n%s' % e
			return None

		current_positions = [round(joint_values[joint_id], 4) for joint_id in joint_ids]
		return current_positions
	# [/MANUAL_FUNC]