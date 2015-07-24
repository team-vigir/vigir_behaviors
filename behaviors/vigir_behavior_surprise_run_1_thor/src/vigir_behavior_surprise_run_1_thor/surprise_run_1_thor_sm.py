#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_surprise_run_1_thor')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.log_state import LogState
from vigir_flexbe_states.plan_affordance_state import PlanAffordanceState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_states.calculation_state import CalculationState
from vigir_flexbe_states.get_template_affordance_state import GetTemplateAffordanceState
from vigir_flexbe_states.finger_configuration_state import FingerConfigurationState
from vigir_flexbe_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from vigir_flexbe_states.footstep_plan_relative_state import FootstepPlanRelativeState
from vigir_flexbe_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from flexbe_states.decision_state import DecisionState
from vigir_flexbe_states.check_current_control_mode_state import CheckCurrentControlModeState
from vigir_flexbe_states.create_step_goal_state import CreateStepGoalState
from vigir_flexbe_states.plan_footsteps_state import PlanFootstepsState
from vigir_flexbe_states.get_template_stand_pose_state import GetTemplateStandPoseState
from vigir_flexbe_states.get_template_pregrasp_state import GetTemplatePregraspState
from vigir_flexbe_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from vigir_flexbe_states.get_template_finger_config_state import GetTemplateFingerConfigState
from vigir_flexbe_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from vigir_flexbe_states.hand_trajectory_state import HandTrajectoryState
from vigir_flexbe_states.get_template_grasp_state import GetTemplateGraspState
from vigir_flexbe_states.look_at_target_state import LookAtTargetState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 01 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class SurpriseRun1THORSM(Behavior):
	'''
	Behavior for the surprise task on run 1 of the DRC Finals.
	'''


	def __init__(self):
		super(SurpriseRun1THORSM, self).__init__()
		self.name = 'Surprise Run 1 THOR'

		# parameters of this behavior
		self.add_parameter('hand_side', 'left')
		self.add_parameter('hand_type', 'vt_hand')
		self.add_parameter('parameter_set', 'drc_step_2D')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		self._pull_displacement = 0

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		pull_affordance = "pull"
		affordance_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == "left" else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		pull_displacement = 0.3 # meters
		# x:383 y:840, x:483 y:490
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.none = None
		_state_machine.userdata.step_back_distance = 1.0 # meters
		_state_machine.userdata.grasp_preference = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		self._pull_displacement = pull_displacement	

		# [/MANUAL_CREATE]

		# x:1033 y:40, x:333 y:90, x:1033 y:190
		_sm_go_to_grasp_0 = OperatableStateMachine(outcomes=['finished', 'failed', 'again'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference'])

		with _sm_go_to_grasp_0:
			# x:33 y:49
			OperatableStateMachine.add('Get_Grasp_Info',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'Inform_Grasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low, 'not_available': Autonomy.Low},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'grasp': 'grasp_pose'})

			# x:40 y:293
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_To_Grasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_waypoints'})

			# x:242 y:292
			OperatableStateMachine.add('Plan_To_Grasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Grasp_Pose', 'incomplete': 'Move_To_Grasp_Pose', 'failed': 'Decide_Which_Grasp'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'waypoints': 'grasp_waypoints', 'hand': 'hand_side', 'frame_id': 'grasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:494 y:175
			OperatableStateMachine.add('Move_To_Grasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Optional_Template_Adjustment', 'failed': 'Decide_Which_Grasp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:226 y:177
			OperatableStateMachine.add('Inform_Grasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:970 y:294
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'again'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:41 y:178
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_pose', 'output_value': 'grasp_frame_id'})

			# x:712 y:78
			OperatableStateMachine.add('Optional_Template_Adjustment',
										OperatorDecisionState(outcomes=["grasp", "pregrasp", "skip"], hint="Consider adjusting the template's pose", suggestion="skip"),
										transitions={'grasp': 'Get_Grasp_Info', 'pregrasp': 'again', 'skip': 'finished'},
										autonomy={'grasp': Autonomy.Full, 'pregrasp': Autonomy.Full, 'skip': Autonomy.High})

			# x:754 y:294
			OperatableStateMachine.add('Decide_Which_Grasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same grasp or the next one?', suggestion='same'),
										transitions={'same': 'Optional_Template_Adjustment', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})


		# x:133 y:390, x:433 y:190, x:983 y:140
		_sm_perform_grasp_1 = OperatableStateMachine(outcomes=['finished', 'failed', 'next'], input_keys=['hand_side', 'grasp_preference', 'template_id', 'pregrasp_pose'], output_keys=['grasp_preference'])

		with _sm_perform_grasp_1:
			# x:68 y:76
			OperatableStateMachine.add('Get_Finger_Configuration',
										GetTemplateFingerConfigState(),
										transitions={'done': 'Close_Fingers', 'failed': 'failed', 'not_available': 'Inform_Closing_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'finger_config': 'finger_config'})

			# x:293 y:328
			OperatableStateMachine.add('Convert_Waypoints',
										CalculationState(calculation=lambda msg: [msg.pose]),
										transitions={'done': 'Plan_Back_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_waypoints'})

			# x:496 y:328
			OperatableStateMachine.add('Plan_Back_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=True, include_torso=False, keep_endeffector_orientation=False, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_Back_To_Pregrasp_Pose', 'incomplete': 'Move_Back_To_Pregrasp_Pose', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Low},
										remapping={'waypoints': 'pregrasp_waypoints', 'hand': 'hand_side', 'frame_id': 'pregrasp_frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:662 y:228
			OperatableStateMachine.add('Move_Back_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Increase_Preference_Index', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:296 y:228
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda pose: pose.header.frame_id),
										transitions={'done': 'Convert_Waypoints'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pregrasp_pose', 'output_value': 'pregrasp_frame_id'})

			# x:673 y:128
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'next'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:81 y:228
			OperatableStateMachine.add('Close_Fingers',
										HandTrajectoryState(hand_type=self.hand_type),
										transitions={'done': 'finished', 'failed': 'Extract_Frame_Id'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'finger_trajectory': 'finger_config', 'hand_side': 'hand_side'})

			# x:490 y:75
			OperatableStateMachine.add('Inform_Closing_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		# x:733 y:190, x:383 y:40
		_sm_go_to_pregrasp_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'grasp_preference', 'template_id'], output_keys=['grasp_preference', 'pregrasp_pose'])

		with _sm_go_to_pregrasp_2:
			# x:27 y:68
			OperatableStateMachine.add('Get_Pregrasp_Info',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp_Pose', 'failed': 'failed', 'not_available': 'Inform_Pregrasp_Failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pregrasp_pose'})

			# x:269 y:153
			OperatableStateMachine.add('Inform_Pregrasp_Failed',
										LogState(text="No grasp choice left!", severity=Logger.REPORT_WARN),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Low})

			# x:537 y:228
			OperatableStateMachine.add('Move_To_Pregrasp_Pose',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:25 y:328
			OperatableStateMachine.add('Increase_Preference_Index',
										CalculationState(calculation=lambda x: x + 1),
										transitions={'done': 'Get_Pregrasp_Info'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp_preference', 'output_value': 'grasp_preference'})

			# x:266 y:228
			OperatableStateMachine.add('Plan_To_Pregrasp_Pose',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp_Pose', 'failed': 'Decide_Which_Pregrasp'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_pose': 'pregrasp_pose', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:266 y:327
			OperatableStateMachine.add('Decide_Which_Pregrasp',
										OperatorDecisionState(outcomes=["same", "next"], hint='Try the same pregrasp or the next one?', suggestion='same'),
										transitions={'same': 'Get_Pregrasp_Info', 'next': 'Increase_Preference_Index'},
										autonomy={'same': Autonomy.High, 'next': Autonomy.High})


		# x:30 y:444, x:162 y:478, x:230 y:478
		_sm_planning_pipeline_3 = OperatableStateMachine(outcomes=['finished', 'failed', 'aborted'], input_keys=['stand_pose'], output_keys=['plan_header'])

		with _sm_planning_pipeline_3:
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


		# x:1103 y:424, x:130 y:478
		_sm_grasp_trigger_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'grasp_preference'])

		with _sm_grasp_trigger_4:
			# x:86 y:72
			OperatableStateMachine.add('Go_to_Pregrasp',
										_sm_go_to_pregrasp_2,
										transitions={'finished': 'Open_Fingers', 'failed': 'Grasp_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:789 y:172
			OperatableStateMachine.add('Perform_Grasp',
										_sm_perform_grasp_1,
										transitions={'finished': 'finished', 'failed': 'Grasp_Manually', 'next': 'Close_Fingers'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'next': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id', 'pregrasp_pose': 'pregrasp_pose'})

			# x:332 y:178
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0.0),
										transitions={'done': 'Go_to_Grasp', 'failed': 'Grasp_Manually'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})

			# x:332 y:28
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1.0),
										transitions={'done': 'Go_to_Pregrasp', 'failed': 'Grasp_Manually'},
										autonomy={'done': Autonomy.High, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})

			# x:324 y:428
			OperatableStateMachine.add('Grasp_Manually',
										OperatorDecisionState(outcomes=["fingers_closed", "abort"], hint="Grasp the object manually, continue when fingers are closed.", suggestion=None),
										transitions={'fingers_closed': 'finished', 'abort': 'failed'},
										autonomy={'fingers_closed': Autonomy.Full, 'abort': Autonomy.Full})

			# x:543 y:172
			OperatableStateMachine.add('Go_to_Grasp',
										_sm_go_to_grasp_0,
										transitions={'finished': 'Perform_Grasp', 'failed': 'Grasp_Manually', 'again': 'Close_Fingers'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'again': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'grasp_preference': 'grasp_preference', 'template_id': 'template_id'})


		# x:30 y:478, x:130 y:478, x:230 y:478
		_sm_walk_to_template_5 = OperatableStateMachine(outcomes=['finished', 'failed', 'aborted'], input_keys=['template_id', 'grasp_preference', 'hand_side'])

		with _sm_walk_to_template_5:
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
										_sm_planning_pipeline_3,
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


		# x:133 y:340, x:383 y:140
		_sm_perform_step_back_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['step_back_distance'])

		with _sm_perform_step_back_6:
			# x:78 y:78
			OperatableStateMachine.add('Plan_Steps_Back',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_BACKWARD),
										transitions={'planned': 'Do_Steps_Back', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'distance': 'step_back_distance', 'plan_header': 'plan_header'})

			# x:74 y:228
			OperatableStateMachine.add('Do_Steps_Back',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'plan_header': 'plan_header'})


		# x:133 y:340, x:333 y:90
		_sm_release_trigger_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'none'])

		with _sm_release_trigger_7:
			# x:82 y:78
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0),
										transitions={'done': 'Take_Hand_Back', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'hand_side': 'hand_side'})

			# x:96 y:178
			OperatableStateMachine.add('Take_Hand_Back',
										LogState(text="Take hand slightly back", severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})


		# x:733 y:240, x:33 y:289
		_sm_pull_trigger_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none'])

		with _sm_pull_trigger_8:
			# x:202 y:28
			OperatableStateMachine.add('Ready_To_Pull',
										LogState(text="Ready to pull the trigger down", severity=Logger.REPORT_INFO),
										transitions={'done': 'Get_Pull_Affordance'},
										autonomy={'done': Autonomy.High})

			# x:192 y:328
			OperatableStateMachine.add('Plan_Pull',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Pull', 'incomplete': 'Execute_Pull', 'failed': 'failed'},
										autonomy={'done': Autonomy.High, 'incomplete': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'affordance': 'affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:176 y:428
			OperatableStateMachine.add('Execute_Pull',
										ExecuteTrajectoryMsgState(controller=affordance_controller),
										transitions={'done': 'Decide_Repeat_Pull', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:183 y:228
			OperatableStateMachine.add('Scale_Pull_Affordance',
										CalculationState(calculation=lambda x: x),
										transitions={'done': 'Plan_Pull'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'affordance', 'output_value': 'affordance'})

			# x:173 y:128
			OperatableStateMachine.add('Get_Pull_Affordance',
										GetTemplateAffordanceState(identifier=pull_affordance),
										transitions={'done': 'Scale_Pull_Affordance', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'affordance'})

			# x:437 y:228
			OperatableStateMachine.add('Decide_Repeat_Pull',
										OperatorDecisionState(outcomes=['done', 'repeat'], hint="Pull further?", suggestion='done'),
										transitions={'done': 'finished', 'repeat': 'Get_Pull_Affordance'},
										autonomy={'done': Autonomy.High, 'repeat': Autonomy.Full})



		with _state_machine:
			# x:73 y:78
			OperatableStateMachine.add('Request_Trigger_Template',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Place trigger template"),
										transitions={'received': 'Decide_Walking', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.Low, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'template_id'})

			# x:337 y:78
			OperatableStateMachine.add('Decide_Walking',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Walk to template?", suggestion="walk"),
										transitions={'walk': 'Walk_To_Template', 'stand': 'Set_Manipulate'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:844 y:322
			OperatableStateMachine.add('Pull_Trigger',
										_sm_pull_trigger_8,
										transitions={'finished': 'Release_Trigger', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:836 y:422
			OperatableStateMachine.add('Release_Trigger',
										_sm_release_trigger_7,
										transitions={'finished': 'Warn_Stand', 'failed': 'Warn_Stand'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'none': 'none'})

			# x:826 y:678
			OperatableStateMachine.add('Go_To_Stand_Pose',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.STAND_POSE, vel_scaling=0.1, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Set_Stand', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'side': 'none'})

			# x:566 y:78
			OperatableStateMachine.add('Set_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Set_Template_Frame', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:858 y:578
			OperatableStateMachine.add('Warn_Stand',
										LogState(text="Will go to stand now", severity=Logger.REPORT_INFO),
										transitions={'done': 'Go_To_Stand_Pose'},
										autonomy={'done': Autonomy.High})

			# x:566 y:678
			OperatableStateMachine.add('Set_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Decide_Step_Back', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:337 y:678
			OperatableStateMachine.add('Decide_Step_Back',
										OperatorDecisionState(outcomes=["walk", "stand"], hint="Step back?", suggestion="walk"),
										transitions={'walk': 'Perform_Step_Back', 'stand': 'finished'},
										autonomy={'walk': Autonomy.High, 'stand': Autonomy.Full})

			# x:77 y:672
			OperatableStateMachine.add('Perform_Step_Back',
										_sm_perform_step_back_6,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'step_back_distance': 'step_back_distance'})

			# x:330 y:172
			OperatableStateMachine.add('Walk_To_Template',
										_sm_walk_to_template_5,
										transitions={'finished': 'Set_Manipulate', 'failed': 'failed', 'aborted': 'Set_Manipulate'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'grasp_preference': 'grasp_preference', 'hand_side': 'hand_side'})

			# x:841 y:222
			OperatableStateMachine.add('Grasp_Trigger',
										_sm_grasp_trigger_4,
										transitions={'finished': 'Pull_Trigger', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'grasp_preference': 'grasp_preference'})

			# x:846 y:128
			OperatableStateMachine.add('Look_At_Trigger',
										LookAtTargetState(),
										transitions={'done': 'Grasp_Trigger'},
										autonomy={'done': Autonomy.Off},
										remapping={'frame': 'template_frame'})

			# x:837 y:28
			OperatableStateMachine.add('Set_Template_Frame',
										CalculationState(calculation=lambda x: "template_tf_%d" % x),
										transitions={'done': 'Look_At_Trigger'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'template_id', 'output_value': 'template_frame'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	def scale_pull_affordance(self, affordance):
		affordance.displacement = math.copysign(self._pull_displacement, affordance.displacement)
		return affordance

	# [/MANUAL_FUNC]
