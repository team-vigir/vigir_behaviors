#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_turn_valve_thor')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.input_state import InputState
from vigir_flexbe_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_states.operator_decision_state import OperatorDecisionState
from vigir_flexbe_states.get_template_pregrasp_state import GetTemplatePregraspState
from vigir_flexbe_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from vigir_flexbe_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from vigir_flexbe_states.get_template_affordance_state import GetTemplateAffordanceState
from vigir_flexbe_states.plan_affordance_state import PlanAffordanceState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.decision_state import DecisionState
from vigir_flexbe_states.look_at_target_state import LookAtTargetState
from flexbe_states.log_state import LogState
from vigir_flexbe_states.footstep_plan_relative_state import FootstepPlanRelativeState
from vigir_flexbe_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from vigir_flexbe_states.get_template_grasp_state import GetTemplateGraspState
from vigir_flexbe_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_states.flexible_calculation_state import FlexibleCalculationState
from vigir_flexbe_states.check_current_control_mode_state import CheckCurrentControlModeState
from vigir_flexbe_states.create_step_goal_state import CreateStepGoalState
from vigir_flexbe_states.plan_footsteps_state import PlanFootstepsState
from vigir_flexbe_states.get_template_stand_pose_state import GetTemplateStandPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

import math

# [/MANUAL_IMPORT]


'''
Created on Mon Apr 06 2015
@author: Philipp Schillinger, Dorothea Koert
'''
class TurnValveTHORSM(Behavior):
	'''
	Lets the robot turn the valve.
	'''


	def __init__(self):
		super(TurnValveTHORSM, self).__init__()
		self.name = 'Turn Valve THOR'

		# parameters of this behavior
		self.add_parameter('hand_type', 'vt_hand')
		self.add_parameter('hand_side', 'right')
		self.add_parameter('parameter_set', 'drc_step_2D')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		self._turn_amount = 0 # set on create
		self._turn_test_amount = 0 # set on create

		# [/MANUAL_INIT]

		# Behavior comments:

		# O 328 125 /Prepare_Manipulation
		# Pregrasp pose is assumed to be in front of the valve

		# O 845 37 /Manipulate_Valve/Perform_Turning
		# Failed outcome used to preempt execution



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		no_valve_collision = True
		turning_affordance = "open"
		turn_amount = 200 # degree
		turn_back_affordance = "close"
		turn_test_amount = 25 # degree
		# x:933 y:490, x:333 y:390
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.default_preference = 0
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.step_back_distance = 1 # meters
		_state_machine.userdata.none = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		self._turn_amount = turn_amount
		self._turn_test_amount = turn_test_amount

		# [/MANUAL_CREATE]

		# x:30 y:478, x:130 y:478, x:230 y:478
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


		# x:483 y:540, x:183 y:290
		_sm_perform_turn_back_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'back_rotation', 'none'])

		with _sm_perform_turn_back_1:
			# x:71 y:78
			OperatableStateMachine.add('Get_Turn_Back_Affordance',
										GetTemplateAffordanceState(identifier=turn_back_affordance),
										transitions={'done': 'Set_Back_Rotation', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'turning_affordance'})

			# x:419 y:228
			OperatableStateMachine.add('Plan_Turn_Back_Affordance',
										PlanAffordanceState(vel_scaling=0.2, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Turn_Back_Affordance', 'incomplete': 'Execute_Turn_Back_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'turning_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:411 y:378
			OperatableStateMachine.add('Execute_Turn_Back_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'finished'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:434 y:78
			OperatableStateMachine.add('Set_Back_Rotation',
										FlexibleCalculationState(calculation=self.set_back_rotation, input_keys=["turning_affordance", "back_rotation"]),
										transitions={'done': 'Plan_Turn_Back_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'turning_affordance': 'turning_affordance', 'back_rotation': 'back_rotation', 'output_value': 'turning_affordance'})


		# x:383 y:40, x:433 y:490
		_sm_perform_turning_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none'], output_keys=['back_rotation'])

		with _sm_perform_turning_2:
			# x:92 y:28
			OperatableStateMachine.add('Adjust_Hand_Pose',
										LogState(text="Make sure hand is in valve", severity=Logger.REPORT_HINT),
										transitions={'done': 'Init_Back_Rotation'},
										autonomy={'done': Autonomy.Full})

			# x:926 y:228
			OperatableStateMachine.add('Plan_Turning_Affordance',
										PlanAffordanceState(vel_scaling=0.3, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Turning_Affordance', 'incomplete': 'Execute_Turning_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'turning_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:495 y:228
			OperatableStateMachine.add('Set_Full_Rotation',
										CalculationState(calculation=self.set_full_rotation_angle),
										transitions={'done': 'Plan_Turning_Affordance'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'turning_affordance', 'output_value': 'turning_affordance'})

			# x:818 y:78
			OperatableStateMachine.add('Execute_Turning_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Accumulate_Rotation', 'failed': 'Accumulate_Rotation'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:534 y:78
			OperatableStateMachine.add('Accumulate_Rotation',
										FlexibleCalculationState(calculation=lambda x: x[0] + self._turn_amount * x[1], input_keys=["back_rotation", "plan_fraction"]),
										transitions={'done': 'Decide_Turn_Again'},
										autonomy={'done': Autonomy.Off},
										remapping={'back_rotation': 'back_rotation', 'plan_fraction': 'plan_fraction', 'output_value': 'back_rotation'})

			# x:287 y:128
			OperatableStateMachine.add('Decide_Turn_Again',
										OperatorDecisionState(outcomes=["extract", "turn"], hint="Turn again or extract hand?", suggestion="turn"),
										transitions={'extract': 'finished', 'turn': 'Decide_If_Test'},
										autonomy={'extract': Autonomy.High, 'turn': Autonomy.Full})

			# x:73 y:228
			OperatableStateMachine.add('Get_Turning_Affordance',
										GetTemplateAffordanceState(identifier=turning_affordance),
										transitions={'done': 'Decide_If_Test', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'turning_affordance'})

			# x:302 y:228
			OperatableStateMachine.add('Decide_If_Test',
										DecisionState(outcomes=["test", "full"], conditions=lambda x: "test" if x == 0 else "full"),
										transitions={'test': 'Set_Test_Rotation', 'full': 'Set_Full_Rotation'},
										autonomy={'test': Autonomy.Low, 'full': Autonomy.Low},
										remapping={'input_value': 'back_rotation'})

			# x:494 y:178
			OperatableStateMachine.add('Set_Test_Rotation',
										CalculationState(calculation=self.set_test_rotation_angle),
										transitions={'done': 'Plan_Test_Turning_Affordance'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'turning_affordance', 'output_value': 'turning_affordance'})

			# x:92 y:128
			OperatableStateMachine.add('Init_Back_Rotation',
										CalculationState(calculation=lambda x: 0),
										transitions={'done': 'Get_Turning_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'none', 'output_value': 'back_rotation'})

			# x:713 y:178
			OperatableStateMachine.add('Plan_Test_Turning_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Turning_Affordance', 'incomplete': 'Execute_Turning_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'turning_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})


		# x:733 y:190, x:433 y:140
		_sm_extract_hand_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'preference', 'none'])

		with _sm_extract_hand_3:
			# x:77 y:55
			OperatableStateMachine.add('Get_Pregrasp',
										GetTemplatePregraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'preference', 'pre_grasp': 'pre_grasp'})

			# x:296 y:328
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=no_valve_collision, include_torso=False, keep_endeffector_orientation=True, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp', 'incomplete': 'Move_To_Pregrasp', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'waypoints': 'waypoints', 'hand': 'hand_side', 'frame_id': 'frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:96 y:328
			OperatableStateMachine.add('Create_Pose_List',
										CalculationState(calculation=lambda x: [x.pose]),
										transitions={'done': 'Plan_To_Pregrasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pre_grasp', 'output_value': 'waypoints'})

			# x:676 y:328
			OperatableStateMachine.add('Move_To_Pregrasp',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:95 y:228
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda x: x.header.frame_id),
										transitions={'done': 'Create_Pose_List'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pre_grasp', 'output_value': 'frame_id'})


		# x:733 y:190, x:433 y:190
		_sm_insert_hand_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'preference', 'none'])

		with _sm_insert_hand_4:
			# x:83 y:78
			OperatableStateMachine.add('Get_Grasp',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'preference', 'grasp': 'grasp'})

			# x:296 y:328
			OperatableStateMachine.add('Plan_To_Grasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=no_valve_collision, include_torso=False, keep_endeffector_orientation=True, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Grasp', 'incomplete': 'Move_To_Grasp', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'waypoints': 'waypoints', 'hand': 'hand_side', 'frame_id': 'frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:96 y:328
			OperatableStateMachine.add('Create_Pose_List',
										CalculationState(calculation=lambda x: [x.pose]),
										transitions={'done': 'Plan_To_Grasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp', 'output_value': 'waypoints'})

			# x:676 y:328
			OperatableStateMachine.add('Move_To_Grasp',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:96 y:228
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda x: x.header.frame_id),
										transitions={'done': 'Create_Pose_List'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp', 'output_value': 'frame_id'})


		# x:933 y:290, x:133 y:340
		_sm_prepare_joint_limit_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'none'])

		with _sm_prepare_joint_limit_5:
			# x:71 y:78
			OperatableStateMachine.add('Get_Turn_Back_Affordance',
										GetTemplateAffordanceState(identifier=turn_back_affordance),
										transitions={'done': 'Set_Back_Rotation', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'turning_affordance'})

			# x:419 y:228
			OperatableStateMachine.add('Plan_Turn_Back_Affordance',
										PlanAffordanceState(vel_scaling=0.2, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Turn_Back_Affordance', 'incomplete': 'Execute_Turn_Back_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'affordance': 'turning_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:411 y:378
			OperatableStateMachine.add('Execute_Turn_Back_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Turn_Further', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:442 y:78
			OperatableStateMachine.add('Set_Back_Rotation',
										CalculationState(calculation=self.set_full_rotation_angle),
										transitions={'done': 'Plan_Turn_Back_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'turning_affordance', 'output_value': 'turning_affordance'})

			# x:687 y:278
			OperatableStateMachine.add('Decide_Turn_Further',
										DecisionState(outcomes=["turn_again", "done"], conditions=lambda x: "turn_again" if x > 0.95 else "done"),
										transitions={'turn_again': 'Plan_Turn_Back_Affordance', 'done': 'finished'},
										autonomy={'turn_again': Autonomy.High, 'done': Autonomy.Low},
										remapping={'input_value': 'plan_fraction'})


		# x:30 y:478, x:130 y:478, x:230 y:478
		_sm_walk_to_template_6 = OperatableStateMachine(outcomes=['finished', 'failed', 'aborted'], input_keys=['template_id', 'grasp_preference', 'hand_side'])

		with _sm_walk_to_template_6:
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


		# x:183 y:590, x:383 y:290
		_sm_manipulate_valve_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'preference', 'none'])

		with _sm_manipulate_valve_7:
			# x:144 y:72
			OperatableStateMachine.add('Insert_Hand',
										_sm_insert_hand_4,
										transitions={'finished': 'Perform_Turning', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'preference': 'preference', 'none': 'none'})

			# x:543 y:422
			OperatableStateMachine.add('Extract_Hand',
										_sm_extract_hand_3,
										transitions={'finished': 'Decide_Repeat', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'preference': 'preference', 'none': 'none'})

			# x:534 y:72
			OperatableStateMachine.add('Perform_Turning',
										_sm_perform_turning_2,
										transitions={'finished': 'Extract_Hand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none', 'back_rotation': 'back_rotation'})

			# x:127 y:272
			OperatableStateMachine.add('Perform_Turn_Back',
										_sm_perform_turn_back_1,
										transitions={'finished': 'Adjust_Hand_Rotation', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'back_rotation': 'back_rotation', 'none': 'none'})

			# x:137 y:428
			OperatableStateMachine.add('Decide_Repeat',
										OperatorDecisionState(outcomes=['insert_again', 'continue'], hint="Continue or adjust wry2 rotation and rotate again", suggestion='insert_again'),
										transitions={'insert_again': 'Perform_Turn_Back', 'continue': 'finished'},
										autonomy={'insert_again': Autonomy.High, 'continue': Autonomy.Full})

			# x:134 y:178
			OperatableStateMachine.add('Adjust_Hand_Rotation',
										OperatorDecisionState(outcomes=['done'], hint="Adjust poking stick rotation", suggestion=None),
										transitions={'done': 'Insert_Hand'},
										autonomy={'done': Autonomy.Full})


		# x:83 y:340, x:333 y:140
		_sm_step_back_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['distance'])

		with _sm_step_back_8:
			# x:28 y:63
			OperatableStateMachine.add('Plan_Steps_Back',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_BACKWARD),
										transitions={'planned': 'Execute_Steps_Back', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'distance': 'distance', 'plan_header': 'plan_header'})

			# x:24 y:163
			OperatableStateMachine.add('Execute_Steps_Back',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'plan_header': 'plan_header'})


		# x:638 y:580, x:230 y:239
		_sm_prepare_manipulation_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'grasp_preference', 'hand_side', 'none'])

		with _sm_prepare_manipulation_9:
			# x:65 y:36
			OperatableStateMachine.add('Go_To_Manipulate',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Set_Template_Frame', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:586 y:463
			OperatableStateMachine.add('Adjust_Hand_Rotation',
										OperatorDecisionState(outcomes=['done'], hint="Adjust poking stick rotation", suggestion=None),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})

			# x:116 y:409
			OperatableStateMachine.add('Get_Pregrasp',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full, 'not_available': Autonomy.Full},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pre_grasp'})

			# x:329 y:178
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'target_pose': 'pre_grasp', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:576 y:178
			OperatableStateMachine.add('Move_To_Pregrasp',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Prepare_Joint_Limit', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.Full},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:577 y:322
			OperatableStateMachine.add('Prepare_Joint_Limit',
										_sm_prepare_joint_limit_5,
										transitions={'finished': 'Adjust_Hand_Rotation', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:53 y:130
			OperatableStateMachine.add('Set_Template_Frame',
										CalculationState(calculation=lambda x: 'template_tf_' + str(x)),
										transitions={'done': 'Look_At_Valve'},
										autonomy={'done': Autonomy.Low},
										remapping={'input_value': 'template_id', 'output_value': 'template_frame'})

			# x:51 y:213
			OperatableStateMachine.add('Look_At_Valve',
										LookAtTargetState(),
										transitions={'done': 'Align_Valve_Log'},
										autonomy={'done': Autonomy.Low},
										remapping={'frame': 'template_frame'})

			# x:55 y:301
			OperatableStateMachine.add('Align_Valve_Log',
										LogState(text="Adjust pose of the valve template", severity=Logger.REPORT_HINT),
										transitions={'done': 'Get_Pregrasp'},
										autonomy={'done': Autonomy.Full})



		with _state_machine:
			# x:35 y:78
			OperatableStateMachine.add('Request_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Place the valve template"),
										transitions={'received': 'Decide_Walking', 'aborted': 'failed', 'no_connection': 'failed', 'data_error': 'failed'},
										autonomy={'received': Autonomy.High, 'aborted': Autonomy.Full, 'no_connection': Autonomy.Full, 'data_error': Autonomy.Full},
										remapping={'data': 'template_id'})

			# x:473 y:72
			OperatableStateMachine.add('Prepare_Manipulation',
										_sm_prepare_manipulation_9,
										transitions={'finished': 'Manipulate_Valve', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'grasp_preference': 'default_preference', 'hand_side': 'hand_side', 'none': 'none'})

			# x:237 y:28
			OperatableStateMachine.add('Decide_Walking',
										OperatorDecisionState(outcomes=['walk', 'skip'], hint="Walk to the valve?", suggestion='skip'),
										transitions={'walk': 'Walk_To_Template', 'skip': 'Prepare_Manipulation'},
										autonomy={'walk': Autonomy.Full, 'skip': Autonomy.Low})

			# x:644 y:557
			OperatableStateMachine.add('Step_Back',
										_sm_step_back_8,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'distance': 'step_back_distance'})

			# x:637 y:428
			OperatableStateMachine.add('Decide_Step_Back',
										OperatorDecisionState(outcomes=["stand", "step_back"], hint="Should the robot step back from the valve?", suggestion="step_back"),
										transitions={'stand': 'finished', 'step_back': 'Step_Back'},
										autonomy={'stand': Autonomy.Full, 'step_back': Autonomy.High})

			# x:738 y:228
			OperatableStateMachine.add('Inform_About_Stand',
										LogState(text="Back to STAND", severity=Logger.REPORT_INFO),
										transitions={'done': 'Go_To_Stand'},
										autonomy={'done': Autonomy.Low})

			# x:733 y:72
			OperatableStateMachine.add('Manipulate_Valve',
										_sm_manipulate_valve_7,
										transitions={'finished': 'Inform_About_Stand', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'default_preference', 'none': 'none'})

			# x:616 y:328
			OperatableStateMachine.add('Go_To_Stand',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'Decide_Step_Back', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Low, 'failed': Autonomy.Full})

			# x:230 y:122
			OperatableStateMachine.add('Walk_To_Template',
										_sm_walk_to_template_6,
										transitions={'finished': 'Prepare_Manipulation', 'failed': 'failed', 'aborted': 'Prepare_Manipulation'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'grasp_preference': 'default_preference', 'hand_side': 'hand_side'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	def set_test_rotation_angle(self, affordance):
		degrees = math.copysign(self._turn_test_amount, affordance.displacement)
		affordance.displacement = math.radians(degrees)
		Logger.loginfo("Turning by %.1f degrees" % degrees)
		return affordance

	def set_full_rotation_angle(self, affordance):
		degrees = math.copysign(self._turn_amount, affordance.displacement)
		affordance.displacement = math.radians(degrees)
		Logger.loginfo("Turning by %.1f degrees" % degrees)
		return affordance

	def set_back_rotation(self, input_keys):
		affordance = input_keys[0]
		back_rotation = input_keys[1]
		degrees = math.copysign(back_rotation, affordance.displacement)
		affordance.displacement = math.radians(degrees)
		Logger.loginfo("Turning back %.1f degrees" % degrees)
		return affordance
	
	# [/MANUAL_FUNC]
