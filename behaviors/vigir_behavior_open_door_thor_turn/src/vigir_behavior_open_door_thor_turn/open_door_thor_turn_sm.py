#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_open_door_thor_turn')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.decision_state import DecisionState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.log_state import LogState
from flexbe_atlas_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_atlas_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_atlas_states.get_template_grasp_state import GetTemplateGraspState
from flexbe_atlas_states.plan_affordance_state import PlanAffordanceState
from flexbe_atlas_states.finger_configuration_state import FingerConfigurationState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from flexbe_atlas_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_atlas_states.get_template_pregrasp_state import GetTemplatePregraspState
from flexbe_atlas_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from flexbe_atlas_states.execute_trajectory_state import ExecuteTrajectoryState
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_atlas_states.look_at_target_state import LookAtTargetState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from flexbe_atlas_states.footstep_plan_relative_state import FootstepPlanRelativeState
from flexbe_atlas_states.check_current_control_mode_state import CheckCurrentControlModeState
from flexbe_atlas_states.create_step_goal_state import CreateStepGoalState
from flexbe_atlas_states.plan_footsteps_state import PlanFootstepsState
from flexbe_atlas_states.get_template_stand_pose_state import GetTemplateStandPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
# [/MANUAL_IMPORT]


'''
Created on Sun May 24 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class OpenDoorTHORTurnSM(Behavior):
	'''
	Open door task by standing sidewards to the door and turning the torso.
	'''


	def __init__(self):
		super(OpenDoorTHORTurnSM, self).__init__()
		self.name = 'Open Door THOR Turn'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')
		self.add_parameter('hand_type', 'vt_hand')
		self.add_parameter('parameter_set', 'drc_step_2D')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 980 304 /Prepare_to_Open
		# Is it worth having a sub behavior for this preference game?

		# O 671 126 /Open_Door/Unlock_Door
		# make sure the handle is turned far enough

		# O 177 29 /Open_Door/Release_Handle/Hand_Back
		# Go back to grasp pose which reverts the pushing affordance



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		handle_down_affordance = 'turnCW' if self.hand_side == 'left' else 'turnCCW'
		door_affordance = 'push'
		handle_up_affordance = 'turnCCW' if self.hand_side == 'left' else 'turnCW'
		turn_threshold = 0.6
		side_sign = 1 if self.hand_side == "left" else -1
		torso_joint_names = ['waist_pan', 'waist_tilt']
		# x:433 y:590, x:333 y:340
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.grasp_preference = 0
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.none = None
		_state_machine.userdata.waypoint_distance = 0.5 # meters
		_state_machine.userdata.torso_center_config = [[0.0, 0.0]]
		_state_machine.userdata.torso_manipulate_handle_config = [[side_sign * 0.92, 0.0]]
		_state_machine.userdata.torso_lidar_config = [[side_sign * 1.46, 0.0]]
		_state_machine.userdata.torso_time = [4.0]
		_state_machine.userdata.away_side = 'right' if self.hand_side == 'left' else 'left'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]


		# [/MANUAL_CREATE]

		# x:833 y:490, x:433 y:190, x:733 y:340
		_sm_planning_pipeline_0 = OperatableStateMachine(outcomes=['finished', 'failed', 'aborted'], input_keys=['stand_pose'], output_keys=['plan_header'])

		with _sm_planning_pipeline_0:
			# x:34 y:57
			OperatableStateMachine.add('Create_Step_Goal',
										CreateStepGoalState(pose_is_pelvis=True),
										transitions={'done': 'Plan_To_Waypoint', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'target_pose': 'stand_pose', 'step_goal': 'step_goal'})

			# x:553 y:481
			OperatableStateMachine.add('Modify_Plan',
										InputState(request=InputState.FOOTSTEP_PLAN_HEADER, message='Modify plan, VALIDATE, and confirm.'),
										transitions={'received': 'finished', 'aborted': 'aborted', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'plan_header'})

			# x:34 y:484
			OperatableStateMachine.add('Plan_To_Waypoint',
										PlanFootstepsState(mode=self.parameter_set),
										transitions={'planned': 'Modify_Plan', 'failed': 'Decide_Replan_without_Collision'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'step_goal': 'step_goal', 'plan_header': 'plan_header'})

			# x:139 y:314
			OperatableStateMachine.add('Decide_Replan_without_Collision',
										OperatorDecisionState(outcomes=['replan', 'fail'], hint='Try replanning without collision avoidance.', suggestion='replan'),
										transitions={'replan': 'Replan_without_Collision', 'fail': 'failed'},
										autonomy={'replan': Autonomy.Low, 'fail': Autonomy.Full})

			# x:319 y:406
			OperatableStateMachine.add('Replan_without_Collision',
										PlanFootstepsState(mode='drc_step_no_collision'),
										transitions={'planned': 'Modify_Plan', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'step_goal': 'step_goal', 'plan_header': 'plan_header'})


		# x:933 y:390, x:433 y:90
		_sm_next_pre_grasp_pose_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'])

		with _sm_next_pre_grasp_pose_1:
			# x:27 y:128
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:246 y:378
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Prerasp_Pose', 'incomplete': 'Move_To_Prerasp_Pose', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:626 y:378
			OperatableStateMachine.add('Move_To_Prerasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:237 y:28
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:46 y:278
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_frame_id'})

			# x:41 y:378
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_waypoints'})

			# x:373 y:170
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Get_Pregrasp_Info'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:479 y:270
			OperatableStateMachine.add('Decide_Which_Pregrasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same pregrasp or the next one?', suggestion='same'),
										transitions={'same': 'Get_Pregrasp_Info', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.Full})


		# x:683 y:190, x:333 y:40
		_sm_back_to_pre_grasp_pose_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'])

		with _sm_back_to_pre_grasp_pose_2:
			# x:27 y:78
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:296 y:278
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'incomplete': 'Move_To_Pregrasp_Pose', 'failed': 'Get_Pregrasp_Info'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:626 y:278
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Get_Pregrasp_Info'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:387 y:78
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:46 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_frame_id'})

			# x:41 y:278
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'grasp_waypoints'})


		# x:883 y:190, x:383 y:90, x:433 y:90
		_sm_go_to_grasp_pose_3 = OperatableStateMachine(outcomes=['finished', 'failed', 'again'], input_keys=['hand_side', 'grasp_preference', 'template_id'])

		with _sm_go_to_grasp_pose_3:
			# x:33 y:178
			OperatableStateMachine.add('Get_Grasp_Info',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'grasp': 'grasp_pose'})

			# x:41 y:378
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Grasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_waypoints'})

			# x:296 y:378
			OperatableStateMachine.add('Plan_To_Grasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Grasp_Pose', 'incomplete': 'Move_To_Grasp_Pose', 'failed': 'Decide_Which_Grasp'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:626 y:378
			OperatableStateMachine.add('Move_To_Grasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Optional_Template_Adjustment', 'failed': 'Decide_Which_Grasp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:137 y:78
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:46 y:278
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_frame_id'})

			# x:612 y:178
			OperatableStateMachine.add('Optional_Template_Adjustment',
										OperatorDecisionState(outcomes=["grasp", "pregrasp", "skip"], hint="Consider adjusting the template's pose", suggestion="skip"),
										transitions={'grasp': 'Get_Grasp_Info', 'pregrasp': 'again', 'skip': 'finished'},
										autonomy={'grasp': Autonomy.Full, 'pregrasp': Autonomy.Full, 'skip': Autonomy.High})

			# x:386 y:278
			OperatableStateMachine.add('Decide_Which_Grasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same grasp or the next one?', suggestion='same'),
										transitions={'same': 'Get_Grasp_Info', 'next': 'again'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.Full})


		# x:133 y:490, x:183 y:40, x:683 y:290
		_sm_go_to_pre_grasp_pose_4 = OperatableStateMachine(outcomes=['finished', 'failed', 'adjust'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference', 'pregrasp_pose', 'template_id'])

		with _sm_go_to_pre_grasp_pose_4:
			# x:77 y:78
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp_Pose', 'failed': 'failed', 'not_available': 'Inform_Pregrasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:330 y:78
			OperatableStateMachine.add('Inform_Pregrasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})

			# x:76 y:378
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:423 y:178
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'adjust'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:79 y:228
			OperatableStateMachine.add('Plan_To_Pregrasp_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'target_pose': 'pregrasp_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:429 y:328
			OperatableStateMachine.add('Decide_Which_Pregrasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same pregrasp or the next one?', suggestion='same'),
										transitions={'same': 'adjust', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.Full})


		# x:733 y:390, x:583 y:40
		_sm_hand_back_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_hand_back_5:
			# x:34 y:28
			OperatableStateMachine.add('Init_Grasp_Preference',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Get_Grasp_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'grasp_preference'})

			# x:380 y:128
			OperatableStateMachine.add('Inform_Pregrasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:476 y:428
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Increase_Preference_Index'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:23 y:428
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Get_Grasp_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:191 y:328
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_Back_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_waypoints'})

			# x:446 y:278
			OperatableStateMachine.add('Plan_Back_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'incomplete': 'Move_To_Pregrasp_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoints': 'pregrasp_waypoints', 'hand': 'hand_side', 'frame_id': 'pregrasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:196 y:228
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_frame_id'})

			# x:33 y:128
			OperatableStateMachine.add('Get_Grasp_Pose',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'Inform_Pregrasp_Failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'grasp': 'pregrasp_pose'})


		# x:133 y:640, x:383 y:390
		_sm_turn_to_handle_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['away_side', 'door_template_id', 'torso_time', 'torso_lidar_config', 'torso_manipulate_handle_config'])

		with _sm_turn_to_handle_6:
			# x:87 y:28
			OperatableStateMachine.add('Set_Template_Frame',
										CalculationState(calculation=lambda x: 'template_tf_' + str(x)),
										transitions={'done': 'Look_At_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'door_template_id', 'output_value': 'template_frame'})

			# x:96 y:128
			OperatableStateMachine.add('Look_At_Handle',
										LookAtTargetState(),
										transitions={'done': 'Take_Arm_Up'},
										autonomy={'done': Autonomy.Low},
										remapping={'frame': 'template_frame'})

			# x:99 y:428
			OperatableStateMachine.add('Align_Door_Log',
										LogState(text="Adjust pose of the door template", severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})

			# x:84 y:328
			OperatableStateMachine.add('Turn_Torso_For_Scan',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_TORSO, joint_names=torso_joint_names),
										transitions={'done': 'Align_Door_Log', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_positions': 'torso_lidar_config', 'time': 'torso_time'})

			# x:81 y:528
			OperatableStateMachine.add('Turn_Torso_To_Handle',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_TORSO, joint_names=torso_joint_names),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_positions': 'torso_manipulate_handle_config', 'time': 'torso_time'})

			# x:76 y:228
			OperatableStateMachine.add('Take_Arm_Up',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE_UP, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Turn_Torso_For_Scan', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'away_side'})


		# x:683 y:590, x:333 y:290
		_sm_place_handle_hand_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['door_template_id', 'hand_side', 'grasp_preference'])

		with _sm_place_handle_hand_7:
			# x:68 y:72
			OperatableStateMachine.add('Go_to_Pre_Grasp_Pose',
										_sm_go_to_pre_grasp_pose_4,
										transitions={'finished': 'Decide_If_Open', 'failed': 'failed', 'adjust': 'Next_Pre_Grasp_Pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'adjust': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:629 y:272
			OperatableStateMachine.add('Go_to_Grasp_Pose',
										_sm_go_to_grasp_pose_3,
										transitions={'finished': 'Decide_If_Grasp', 'failed': 'failed', 'again': 'Back_to_Pre_Grasp_Pose'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'again': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id'})

			# x:432 y:178
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0),
										transitions={'done': 'Go_to_Grasp_Pose', 'failed': 'Go_to_Grasp_Pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'hand_side': 'hand_side'})

			# x:63 y:472
			OperatableStateMachine.add('Back_to_Pre_Grasp_Pose',
										_sm_back_to_pre_grasp_pose_2,
										transitions={'finished': 'Increase_Preference_Index', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id'})

			# x:648 y:428
			OperatableStateMachine.add('Decide_If_Grasp',
										DecisionState(outcomes=["grasp", "push"], conditions=lambda x: "grasp" if x == 0 else "push"),
										transitions={'grasp': 'Close_Fingers', 'push': 'finished'},
										autonomy={'grasp': Autonomy.Low, 'push': Autonomy.High},
										remapping={'input_value': 'grasp_preference'})

			# x:432 y:478
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:371 y:22
			OperatableStateMachine.add('Next_Pre_Grasp_Pose',
										_sm_next_pre_grasp_pose_1,
										transitions={'finished': 'Decide_If_Open', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'door_template_id'})

			# x:73 y:278
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Go_to_Pre_Grasp_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:650 y:78
			OperatableStateMachine.add('Decide_If_Open',
										DecisionState(outcomes=["open", "push"], conditions=lambda x: "open" if x == 0 else "push"),
										transitions={'open': 'Open_Fingers', 'push': 'Go_to_Grasp_Pose'},
										autonomy={'open': Autonomy.Low, 'push': Autonomy.High},
										remapping={'input_value': 'grasp_preference'})


		# x:1041 y:400, x:987 y:18
		_sm_unlock_door_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'none', 'door_template_id'], output_keys=['turn_fraction'])

		with _sm_unlock_door_8:
			# x:64 y:28
			OperatableStateMachine.add('Get_Handle_Affordance_Down',
										GetTemplateAffordanceState(identifier=handle_down_affordance),
										transitions={'done': 'Plan_Turn_Handle_Down', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:992 y:178
			OperatableStateMachine.add('Plan_Push_Door',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Push_Door', 'incomplete': 'Push_Door', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'door_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:973 y:78
			OperatableStateMachine.add('Get_Push_Affordance',
										GetTemplateAffordanceState(identifier=door_affordance),
										transitions={'done': 'Plan_Push_Door', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'affordance': 'door_affordance'})

			# x:277 y:78
			OperatableStateMachine.add('Plan_Turn_Handle_Down',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Store_Turn_Down', 'incomplete': 'Decide_Execute_Incomplete', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'handle_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:676 y:78
			OperatableStateMachine.add('Turn_Handle',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Get_Push_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:976 y:278
			OperatableStateMachine.add('Push_Door',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:71 y:263
			OperatableStateMachine.add('Get_Handle_Affordance_Up',
										GetTemplateAffordanceState(identifier=handle_up_affordance),
										transitions={'done': 'Plan_Turn_Handle_Up', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:284 y:270
			OperatableStateMachine.add('Plan_Turn_Handle_Up',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Store_Turn_Up', 'incomplete': 'Store_Turn_Up', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'handle_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:70 y:178
			OperatableStateMachine.add('Decide_Execute_Incomplete',
										DecisionState(outcomes=["up", "down"], conditions=lambda x: "down" if x > turn_threshold else "up"),
										transitions={'up': 'Get_Handle_Affordance_Up', 'down': 'Store_Turn_Down'},
										autonomy={'up': Autonomy.High, 'down': Autonomy.Low},
										remapping={'input_value': 'plan_fraction'})

			# x:494 y:78
			OperatableStateMachine.add('Store_Turn_Down',
										CalculationState(calculation=lambda x: x),
										transitions={'done': 'Turn_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'plan_fraction', 'output_value': 'turn_fraction'})

			# x:501 y:178
			OperatableStateMachine.add('Store_Turn_Up',
										CalculationState(calculation=lambda x: -x),
										transitions={'done': 'Turn_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'plan_fraction', 'output_value': 'turn_fraction'})


		# x:788 y:561, x:133 y:390
		_sm_release_handle_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none', 'turn_fraction', 'grasp_preference'])

		with _sm_release_handle_9:
			# x:33 y:120
			OperatableStateMachine.add('Decide_Turn_Direction',
										DecisionState(outcomes=["up", "down"], conditions=lambda x: "up" if x > 0 else "down"),
										transitions={'up': 'Get_Handle_Affordance_Up', 'down': 'Get_Handle_Affordance_Down'},
										autonomy={'up': Autonomy.Low, 'down': Autonomy.Low},
										remapping={'input_value': 'turn_fraction'})

			# x:744 y:372
			OperatableStateMachine.add('Hand_Back',
										_sm_hand_back_5,
										transitions={'finished': 'Close_Fingers', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})

			# x:742 y:128
			OperatableStateMachine.add('Plan_Turn_Handle',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Turn_Handle', 'incomplete': 'Turn_Handle', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'handle_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:926 y:128
			OperatableStateMachine.add('Turn_Handle',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_If_Open', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:782 y:278
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0.0),
										transitions={'done': 'Decide_Retract_Hand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:455 y:128
			OperatableStateMachine.add('Reduce_Affordance_Displacement',
										FlexibleCalculationState(calculation=self.scale_affordance, input_keys=["affordance", "fraction"]),
										transitions={'done': 'Plan_Turn_Handle'},
										autonomy={'done': Autonomy.Off},
										remapping={'affordance': 'handle_affordance', 'fraction': 'turn_fraction', 'output_value': 'scaled_affordance'})

			# x:214 y:178
			OperatableStateMachine.add('Get_Handle_Affordance_Down',
										GetTemplateAffordanceState(identifier=handle_down_affordance),
										transitions={'done': 'Reduce_Affordance_Displacement', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:221 y:78
			OperatableStateMachine.add('Get_Handle_Affordance_Up',
										GetTemplateAffordanceState(identifier=handle_up_affordance),
										transitions={'done': 'Reduce_Affordance_Displacement', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'handle_affordance'})

			# x:936 y:378
			OperatableStateMachine.add('Decide_Retract_Hand',
										OperatorDecisionState(outcomes=['keep', 'back'], hint="Take hand back from handle?", suggestion='back'),
										transitions={'keep': 'Close_Fingers', 'back': 'Hand_Back'},
										autonomy={'keep': Autonomy.Full, 'back': Autonomy.Low})

			# x:950 y:213
			OperatableStateMachine.add('Decide_If_Open',
										DecisionState(outcomes=["open", "retract"], conditions=lambda x: "open" if x == 0 else "retract"),
										transitions={'open': 'Open_Fingers', 'retract': 'Decide_Retract_Hand'},
										autonomy={'open': Autonomy.Low, 'retract': Autonomy.High},
										remapping={'input_value': 'grasp_preference'})

			# x:782 y:478
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'hand_side': 'hand_side'})


		# x:883 y:640, x:433 y:390, x:1133 y:390
		_sm_walk_to_template_10 = OperatableStateMachine(outcomes=['finished', 'failed', 'aborted'], input_keys=['template_id', 'grasp_preference', 'hand_side'])

		with _sm_walk_to_template_10:
			# x:265 y:28
			OperatableStateMachine.add('Decide_Request_Template',
										DecisionState(outcomes=['request', 'continue'], conditions=lambda x: 'continue' if x is not None else 'request'),
										transitions={'request': 'Request_Template', 'continue': 'Get_Stand_Pose'},
										autonomy={'request': Autonomy.Low, 'continue': Autonomy.Off},
										remapping={'input_value': 'template_id'})

			# x:1033 y:106
			OperatableStateMachine.add('Increment_Stand_Pose',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Inform_About_Retry'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:1162 y:29
			OperatableStateMachine.add('Inform_About_Retry',
										LogState(text="Stand pose choice failed. Trying again.", severity=Logger.REPORT_INFO),
										transitions={'done': 'Get_Stand_Pose'},
										autonomy={'done': Autonomy.Off})

			# x:567 y:118
			OperatableStateMachine.add('Inform_About_Fail',
										LogState(text="Unable to find a suitable stand pose for the template.", severity=Logger.REPORT_WARN),
										transitions={'done': 'Decide_Repeat_Request'},
										autonomy={'done': Autonomy.Off})

			# x:554 y:274
			OperatableStateMachine.add('Get_Goal_from_Operator',
										InputState(request=InputState.WAYPOINT_GOAL_POSE, message="Provide a waypoint in front of the template."),
										transitions={'received': 'Walk_To_Waypoint', 'aborted': 'aborted', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'plan_header'})

			# x:279 y:110
			OperatableStateMachine.add('Request_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Specify target template"),
										transitions={'received': 'Get_Stand_Pose', 'aborted': 'aborted', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'template_id'})

			# x:825 y:461
			OperatableStateMachine.add('Wait_For_Stand',
										CheckCurrentControlModeState(target_mode=CheckCurrentControlModeState.STAND, wait=True),
										transitions={'correct': 'finished', 'incorrect': 'failed'},
										autonomy={'correct': Autonomy.Low, 'incorrect': Autonomy.Full},
										remapping={'control_mode': 'control_mode'})

			# x:1143 y:277
			OperatableStateMachine.add('Decide_Stand_Preference',
										OperatorDecisionState(outcomes=["same", "next", "abort"], hint="Same or next stand pose?", suggestion="next"),
										transitions={'same': 'Inform_About_Retry', 'next': 'Increment_Stand_Pose', 'abort': 'aborted'},
										autonomy={'same': Autonomy.Full, 'next': Autonomy.Full, 'abort': Autonomy.Full})

			# x:842 y:152
			OperatableStateMachine.add('Planning_Pipeline',
										_sm_planning_pipeline_0,
										transitions={'finished': 'Walk_To_Waypoint', 'failed': 'Decide_Stand_Preference', 'aborted': 'Decide_Stand_Preference'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'stand_pose': 'stand_pose', 'plan_header': 'plan_header'})

			# x:833 y:276
			OperatableStateMachine.add('Walk_To_Waypoint',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Wait_For_Stand', 'failed': 'Decide_Stand_Preference'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.Full},
										remapping={'plan_header': 'plan_header'})

			# x:554 y:195
			OperatableStateMachine.add('Decide_Repeat_Request',
										OperatorDecisionState(outcomes=['repeat_id', 'request_goal'], hint=None, suggestion=None),
										transitions={'repeat_id': 'Request_Template', 'request_goal': 'Get_Goal_from_Operator'},
										autonomy={'repeat_id': Autonomy.Low, 'request_goal': Autonomy.High})

			# x:547 y:27
			OperatableStateMachine.add('Get_Stand_Pose',
										GetTemplateStandPoseState(),
										transitions={'done': 'Planning_Pipeline', 'failed': 'Inform_About_Fail', 'not_available': 'Inform_About_Fail'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'stand_pose': 'stand_pose'})


		# x:533 y:90, x:273 y:457
		_sm_step_through_11 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['distance', 'hand_side'])

		with _sm_step_through_11:
			# x:247 y:78
			OperatableStateMachine.add('Decide_Direction',
										DecisionState(outcomes=["left", "right", "done"], conditions=lambda x: self.hand_side),
										transitions={'left': 'Plan_Steps_Left', 'right': 'Plan_Steps_Right', 'done': 'finished'},
										autonomy={'left': Autonomy.Low, 'right': Autonomy.Low, 'done': Autonomy.Full},
										remapping={'input_value': 'hand_side'})

			# x:224 y:278
			OperatableStateMachine.add('Execute_Steps_Through',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'Decide_Direction', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'plan_header': 'plan_header'})

			# x:378 y:178
			OperatableStateMachine.add('Plan_Steps_Right',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_RIGHT),
										transitions={'planned': 'Execute_Steps_Through', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'distance': 'distance', 'plan_header': 'plan_header'})

			# x:78 y:178
			OperatableStateMachine.add('Plan_Steps_Left',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_LEFT),
										transitions={'planned': 'Execute_Steps_Through', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'distance': 'distance', 'plan_header': 'plan_header'})


		# x:733 y:640, x:92 y:675
		_sm_open_door_12 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none', 'door_template_id', 'hand_side', 'grasp_preference', 'torso_time', 'torso_lidar_config', 'torso_center_config', 'torso_manipulate_handle_config', 'away_side'])

		with _sm_open_door_12:
			# x:37 y:78
			OperatableStateMachine.add('Decide_Turn_Torso',
										OperatorDecisionState(outcomes=["turn", "straight"], hint="Turn the torso to the door?", suggestion="turn"),
										transitions={'turn': 'Turn_To_Handle', 'straight': 'Place_Handle_Hand'},
										autonomy={'turn': Autonomy.High, 'straight': Autonomy.Full})

			# x:637 y:172
			OperatableStateMachine.add('Release_Handle',
										_sm_release_handle_9,
										transitions={'finished': 'Prepare_For_Turning', 'failed': 'Log_Remove_Hand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'door_template_id', 'hand_side': 'hand_side', 'none': 'none', 'turn_fraction': 'turn_fraction', 'grasp_preference': 'grasp_preference'})

			# x:841 y:178
			OperatableStateMachine.add('Log_Remove_Hand',
										LogState(text='Remove hand from handle', severity=Logger.REPORT_HINT),
										transitions={'done': 'Prepare_For_Turning'},
										autonomy={'done': Autonomy.Full})

			# x:444 y:122
			OperatableStateMachine.add('Unlock_Door',
										_sm_unlock_door_8,
										transitions={'finished': 'Ask_for_Retry', 'failed': 'Ask_for_Retry'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'none': 'none', 'door_template_id': 'door_template_id', 'turn_fraction': 'turn_fraction'})

			# x:637 y:28
			OperatableStateMachine.add('Ask_for_Retry',
										OperatorDecisionState(outcomes=["release_handle", "retry"], hint="Now release handle?", suggestion="release_handle"),
										transitions={'release_handle': 'Release_Handle', 'retry': 'Log_Try_To_Open'},
										autonomy={'release_handle': Autonomy.Low, 'retry': Autonomy.Full})

			# x:227 y:122
			OperatableStateMachine.add('Place_Handle_Hand',
										_sm_place_handle_hand_7,
										transitions={'finished': 'Log_Try_To_Open', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'door_template_id': 'door_template_id', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference'})

			# x:637 y:278
			OperatableStateMachine.add('Prepare_For_Turning',
										LogState(text='Make sure centering torso is possible', severity=Logger.REPORT_HINT),
										transitions={'done': 'Turn_Torso_Center'},
										autonomy={'done': Autonomy.Full})

			# x:386 y:228
			OperatableStateMachine.add('Turn_Torso_Center',
										ExecuteTrajectoryState(controller=ExecuteTrajectoryState.CONTROLLER_TORSO, joint_names=torso_joint_names),
										transitions={'done': 'Look_Straight', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_positions': 'torso_center_config', 'time': 'torso_time'})

			# x:376 y:528
			OperatableStateMachine.add('Push_Door',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.DOOR_PUSH_SIDE, vel_scaling=0.05, ignore_collisions=False, link_paddings={}, is_cartesian=True),
										transitions={'done': 'Arms_To_Stand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:476 y:628
			OperatableStateMachine.add('Arms_To_Stand',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'none'})

			# x:526 y:378
			OperatableStateMachine.add('Look_To_Door',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.HEAD_SIDE, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=True),
										transitions={'done': 'Decide_Push_Door', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'hand_side'})

			# x:36 y:172
			OperatableStateMachine.add('Turn_To_Handle',
										_sm_turn_to_handle_6,
										transitions={'finished': 'Place_Handle_Hand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'away_side': 'away_side', 'door_template_id': 'door_template_id', 'torso_time': 'torso_time', 'torso_lidar_config': 'torso_lidar_config', 'torso_manipulate_handle_config': 'torso_manipulate_handle_config'})

			# x:443 y:28
			OperatableStateMachine.add('Log_Try_To_Open',
										LogState(text='Will now try to open', severity=Logger.REPORT_INFO),
										transitions={'done': 'Unlock_Door'},
										autonomy={'done': Autonomy.High})

			# x:387 y:428
			OperatableStateMachine.add('Decide_Push_Door',
										OperatorDecisionState(outcomes=['push', 'walk'], hint="Push the door open?", suggestion='walk'),
										transitions={'push': 'Push_Door', 'walk': 'Arms_To_Stand'},
										autonomy={'push': Autonomy.Full, 'walk': Autonomy.Low})

			# x:346 y:328
			OperatableStateMachine.add('Look_Straight',
										LookAtTargetState(),
										transitions={'done': 'Look_To_Door'},
										autonomy={'done': Autonomy.Low},
										remapping={'frame': 'none'})



		with _state_machine:
			# x:82 y:28
			OperatableStateMachine.add('Get_Door_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the DOOR template."),
										transitions={'received': 'Decide_Walking', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.High, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'door_template_id'})

			# x:616 y:478
			OperatableStateMachine.add('Go_To_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Step_Through', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:316 y:128
			OperatableStateMachine.add('Manipulate_On',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Open_Door', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:644 y:122
			OperatableStateMachine.add('Open_Door',
										_sm_open_door_12,
										transitions={'finished': 'Wait_For_Gather_Data', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'door_template_id': 'door_template_id', 'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'torso_time': 'torso_time', 'torso_lidar_config': 'torso_lidar_config', 'torso_center_config': 'torso_center_config', 'torso_manipulate_handle_config': 'torso_manipulate_handle_config', 'away_side': 'away_side'})

			# x:634 y:278
			OperatableStateMachine.add('Wait_For_Gather_Data',
										LogState(text='Gather data from inside', severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_To_Stand'},
										autonomy={'done': Autonomy.Full})

			# x:87 y:128
			OperatableStateMachine.add('Decide_Walking',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Walk to the door?", suggestion="stand"),
										transitions={'walk': 'Walk_To_Template', 'stand': 'Manipulate_On'},
										autonomy={'walk': Autonomy.Full, 'stand': Autonomy.Low})

			# x:394 y:472
			OperatableStateMachine.add('Step_Through',
										_sm_step_through_11,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'distance': 'waypoint_distance', 'hand_side': 'hand_side'})

			# x:80 y:322
			OperatableStateMachine.add('Walk_To_Template',
										_sm_walk_to_template_10,
										transitions={'finished': 'Manipulate_On', 'failed': 'failed', 'aborted': 'Manipulate_On'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'template_id': 'door_template_id', 'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	def calc_open_finger_traj(self, pre_grasp):
		'''Creates a JointTrajectory msg that corresponds to the open fingers configuration (all zeros).

		Uses the grasp pose message to infer the correct joint names.
		'''

		# Set the finger joint configuration that corresponds to an open hand
		open_config = JointTrajectoryPoint()
		open_config.positions = [0.0] * 11
		open_config.time_from_start = rospy.Duration.from_sec(0.1)

		open_fingers_traj = JointTrajectory()
		open_fingers_traj.points.append(open_config)
		open_fingers_traj.joint_names = pre_grasp.pre_grasp_posture.joint_names

		print(open_fingers_traj)

		return open_fingers_traj
		
	def scale_affordance(self, input_list):
		affordance = input_list[0]
		fraction = input_list[1]
		affordance.displacement *= abs(fraction)
		return affordance
	
	# [/MANUAL_FUNC]
