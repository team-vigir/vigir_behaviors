#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('vigir_behavior_turn_valve')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from flexbe_states.log_state import LogState
from flexbe_atlas_states.get_template_affordance_state import GetTemplateAffordanceState
from flexbe_atlas_states.plan_affordance_state import PlanAffordanceState
from flexbe_atlas_states.execute_trajectory_msg_state import ExecuteTrajectoryMsgState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexbe_states.calculation_state import CalculationState
from flexbe_atlas_states.plan_endeffector_pose_state import PlanEndeffectorPoseState
from flexbe_atlas_states.get_template_grasp_state import GetTemplateGraspState
from flexbe_atlas_states.plan_endeffector_cartesian_waypoints_state import PlanEndeffectorCartesianWaypointsState
from flexbe_states.decision_state import DecisionState
from flexbe_atlas_states.get_template_pregrasp_state import GetTemplatePregraspState
from flexbe_atlas_states.footstep_plan_relative_state import FootstepPlanRelativeState
from flexbe_atlas_states.execute_step_plan_action_state import ExecuteStepPlanActionState
from vigir_behavior_walk_to_template.walk_to_template_sm import WalktoTemplateSM
from flexbe_states.input_state import InputState
from flexbe_atlas_states.change_control_mode_action_state import ChangeControlModeActionState
from flexbe_atlas_states.moveit_predefined_pose_state import MoveitPredefinedPoseState
from flexbe_atlas_states.finger_configuration_state import FingerConfigurationState
from flexbe_atlas_states.tilt_head_state import TiltHeadState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

import math

from geometry_msgs.msg import PoseStamped

# [/MANUAL_IMPORT]


'''
Created on Mon Apr 06 2015
@author: Philipp Schillinger
'''
class TurnValveSM(Behavior):
	'''
	Lets the robot walk to and turn the valve.
	'''


	def __init__(self):
		super(TurnValveSM, self).__init__()
		self.name = 'Turn Valve'

		# parameters of this behavior
		self.add_parameter('hand_type', 'robotiq')
		self.add_parameter('hand_side', 'left')

		# references to used behaviors
		self.add_behavior(WalktoTemplateSM, 'Walk to Template')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		self._turn_amount = 0 # set on create
		self._keep_orientation = None # True when using the stick

		# [/MANUAL_INIT]

		# Behavior comments:

		# O 804 401 
		# Not used with poking stick

		# O 499 240 /Retract_Arms
		# Not used with poking stick



	def create(self):
		arm_controller = ExecuteTrajectoryMsgState.CONTROLLER_LEFT_ARM if self.hand_side == 'left' else ExecuteTrajectoryMsgState.CONTROLLER_RIGHT_ARM
		no_valve_collision = True
		turn_amount = 400 # degrees (empirical)
		keep_orientation = True # for poking stick
		# x:1227 y:469, x:290 y:544
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.default_preference = 0
		_state_machine.userdata.hand_side = self.hand_side
		_state_machine.userdata.step_back_distance = 0.30 # meters
		_state_machine.userdata.none = None
		_state_machine.userdata.template_id = 0 # adjust before re-runs!
		_state_machine.userdata.torso_center = 'same'
		_state_machine.userdata.stick_reference = None # hardcoded

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		self._turn_amount = turn_amount
		self._keep_orientation = keep_orientation

		# Use the poking stick as the reference point for circular affordance
		stick_reference_point = PoseStamped()
		stick_reference_point.header.frame_id = 'l_hand'
		stick_reference_point.pose.position.y = -0.095
		stick_reference_point.pose.orientation.w = 1.0
		_state_machine.userdata.stick_reference = stick_reference_point

		# [/MANUAL_CREATE]

		# x:841 y:312, x:634 y:50
		_sm_go_to_grasp_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'hand_side', 'preference'])

		with _sm_go_to_grasp_0:
			# x:36 y:46
			OperatableStateMachine.add('Get_Grasp',
										GetTemplateGraspState(),
										transitions={'done': 'Extract_Frame_Id', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Low, 'not_available': Autonomy.Low},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'preference', 'grasp': 'grasp'})

			# x:248 y:134
			OperatableStateMachine.add('Plan_To_Grasp',
										PlanEndeffectorCartesianWaypointsState(ignore_collisions=no_valve_collision, include_torso=False, keep_endeffector_orientation=True, allow_incomplete_plans=True, vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Grasp', 'incomplete': 'Move_To_Grasp', 'failed': 'Replan_if_Incomplete'},
										autonomy={'planned': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'waypoints': 'waypoints', 'hand': 'hand_side', 'frame_id': 'frame_id', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:42 y:266
			OperatableStateMachine.add('Create_Pose_List',
										CalculationState(calculation=lambda x: [x.pose]),
										transitions={'done': 'Plan_To_Grasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp', 'output_value': 'waypoints'})

			# x:576 y:135
			OperatableStateMachine.add('Move_To_Grasp',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Replan_if_Incomplete', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:274 y:307
			OperatableStateMachine.add('Notify_Incomplete_Plan',
										LogState(text='Incomplete Plan! Re-planning...', severity=Logger.REPORT_HINT),
										transitions={'done': 'Plan_To_Grasp'},
										autonomy={'done': Autonomy.Off})

			# x:585 y:307
			OperatableStateMachine.add('Replan_if_Incomplete',
										DecisionState(outcomes=['replan', 'continue'], conditions=lambda x: 'replan' if x < 0.9 else 'continue'),
										transitions={'replan': 'Notify_Incomplete_Plan', 'continue': 'finished'},
										autonomy={'replan': Autonomy.Low, 'continue': Autonomy.High},
										remapping={'input_value': 'plan_fraction'})

			# x:42 y:167
			OperatableStateMachine.add('Extract_Frame_Id',
										CalculationState(calculation=lambda x: x.header.frame_id),
										transitions={'done': 'Create_Pose_List'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'grasp', 'output_value': 'frame_id'})


		# x:174 y:319, x:443 y:229
		_sm_extract_hand_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_extract_hand_1:
			# x:101 y:81
			OperatableStateMachine.add('Get_Extract_Affordance',
										GetTemplateAffordanceState(identifier='extract'),
										transitions={'done': 'Plan_Extract_Affordance', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'extract_affordance'})

			# x:394 y:82
			OperatableStateMachine.add('Plan_Extract_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Extract_Affordance', 'incomplete': 'Execute_Extract_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'extract_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:627 y:224
			OperatableStateMachine.add('Execute_Extract_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Extract_More', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:394 y:313
			OperatableStateMachine.add('Decide_Extract_More',
										OperatorDecisionState(outcomes=['extract_more', 'skip'], hint='Extract more if the fingers are not out yet.', suggestion='skip'),
										transitions={'extract_more': 'Get_Extract_Affordance', 'skip': 'finished'},
										autonomy={'extract_more': Autonomy.High, 'skip': Autonomy.Low})


		# x:744 y:263
		_sm_insert_hand_2 = OperatableStateMachine(outcomes=['finished'], input_keys=['hand_side', 'template_id', 'none'])

		with _sm_insert_hand_2:
			# x:84 y:72
			OperatableStateMachine.add('Get_Insert_Affordance',
										GetTemplateAffordanceState(identifier='insert'),
										transitions={'done': 'Plan_Insert_Affordance', 'failed': 'Decide_Insert_More', 'not_available': 'Decide_Insert_More'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'insert_affordance'})

			# x:342 y:73
			OperatableStateMachine.add('Plan_Insert_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Insert_Affordance', 'incomplete': 'Execute_Insert_Affordance', 'failed': 'Decide_Insert_More'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'insert_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:665 y:73
			OperatableStateMachine.add('Execute_Insert_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Insert_More', 'failed': 'Decide_Insert_More'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:345 y:257
			OperatableStateMachine.add('Decide_Insert_More',
										OperatorDecisionState(outcomes=['insert_more', 'skip'], hint='Insert more if the fingers are not inside the valve.', suggestion='skip'),
										transitions={'insert_more': 'Get_Insert_Affordance', 'skip': 'finished'},
										autonomy={'insert_more': Autonomy.High, 'skip': Autonomy.Low})


		# x:610 y:263
		_sm_adjust_wrist_3 = OperatableStateMachine(outcomes=['finished'], input_keys=['template_id', 'hand_side', 'none'])

		with _sm_adjust_wrist_3:
			# x:84 y:72
			OperatableStateMachine.add('Get_Close_Affordance',
										GetTemplateAffordanceState(identifier='close'),
										transitions={'done': 'Plan_Close_Affordance', 'failed': 'Adjust_Hand_Rotation', 'not_available': 'Adjust_Hand_Rotation'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'close_affordance'})

			# x:342 y:73
			OperatableStateMachine.add('Plan_Close_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Close_Affordance', 'incomplete': 'Execute_Close_Affordance', 'failed': 'Adjust_Hand_Rotation'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'close_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:665 y:73
			OperatableStateMachine.add('Execute_Close_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Adjust_Hand_Rotation', 'failed': 'Adjust_Hand_Rotation'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:345 y:257
			OperatableStateMachine.add('Adjust_Hand_Rotation',
										OperatorDecisionState(outcomes=['done'], hint="Adjust wry2 rotation (near opposing joint limit)", suggestion=None),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Full})


		# x:605 y:445, x:389 y:143
		_sm_turn_valve_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['stick_reference', 'template_id', 'hand_side'])

		with _sm_turn_valve_4:
			# x:109 y:56
			OperatableStateMachine.add('Get_Turning_Affordance',
										GetTemplateAffordanceState(identifier='open'),
										transitions={'done': 'Set_Rotation', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'turning_affordance'})

			# x:111 y:292
			OperatableStateMachine.add('Plan_Turning_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Turning_Affordance', 'incomplete': 'Execute_Turning_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'turning_affordance', 'hand': 'hand_side', 'reference_point': 'stick_reference', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:127 y:173
			OperatableStateMachine.add('Set_Rotation',
										CalculationState(calculation=self.reduce_rotation_angle),
										transitions={'done': 'Plan_Turning_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'turning_affordance', 'output_value': 'turning_affordance'})

			# x:526 y:292
			OperatableStateMachine.add('Execute_Turning_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Keep_Turning', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:324 y:437
			OperatableStateMachine.add('Decide_Keep_Turning',
										OperatorDecisionState(outcomes=['done', 'turn_more'], hint='Check whether the valve is turned enough', suggestion='done'),
										transitions={'done': 'finished', 'turn_more': 'Plan_Turning_Affordance'},
										autonomy={'done': Autonomy.High, 'turn_more': Autonomy.High})


		# x:175 y:378, x:413 y:156
		_sm_face_valve_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side'])

		with _sm_face_valve_5:
			# x:126 y:29
			OperatableStateMachine.add('Go_to_MANIPULATE',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Tilt_Head_Down', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:148 y:262
			OperatableStateMachine.add('Tilt_Head_Down',
										TiltHeadState(desired_tilt=TiltHeadState.DOWN_45),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High})


		# x:660 y:473, x:418 y:191
		_sm_retract_arms_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['hand_side', 'torso_center'])

		with _sm_retract_arms_6:
			# x:112 y:28
			OperatableStateMachine.add('Set_MANIPULATE',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.MANIPULATE),
										transitions={'changed': 'Move_to_Stand_Posture', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:118 y:360
			OperatableStateMachine.add('Move_to_Stand_Posture',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.SINGLE_ARM_STAND, vel_scaling=0.2, ignore_collisions=False, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Center_Torso', 'failed': 'Move_to_Stand_Posture'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'side': 'hand_side'})

			# x:359 y:464
			OperatableStateMachine.add('Set_STAND',
										ChangeControlModeActionState(target_mode=ChangeControlModeActionState.STAND),
										transitions={'changed': 'finished', 'failed': 'failed'},
										autonomy={'changed': Autonomy.Off, 'failed': Autonomy.High})

			# x:118 y:464
			OperatableStateMachine.add('Center_Torso',
										MoveitPredefinedPoseState(target_pose=MoveitPredefinedPoseState.TURN_TORSO_CENTER_POSE, vel_scaling=0.2, ignore_collisions=True, link_paddings={}, is_cartesian=False),
										transitions={'done': 'Set_STAND', 'failed': 'Center_Torso'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'side': 'torso_center'})

			# x:505 y:87
			OperatableStateMachine.add('Open_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=0.0),
										transitions={'done': 'Close_Fingers', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})

			# x:505 y:185
			OperatableStateMachine.add('Close_Fingers',
										FingerConfigurationState(hand_type=self.hand_type, configuration=1.0),
										transitions={'done': 'Close_Fingers', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'hand_side': 'hand_side'})


		# x:495 y:47, x:187 y:229
		_sm_request_template_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['none'], output_keys=['template_id'])

		with _sm_request_template_7:
			# x:155 y:42
			OperatableStateMachine.add('Request_Template_ID',
										InputState(request=InputState.SELECTED_OBJECT_ID, message="Provide the ID of the placed template."),
										transitions={'received': 'finished', 'aborted': 'failed', 'no_connection': 'Log_No_Connection', 'data_error': 'Log_Data_Error'},
										autonomy={'received': Autonomy.Off, 'aborted': Autonomy.High, 'no_connection': Autonomy.Low, 'data_error': Autonomy.Low},
										remapping={'data': 'template_id'})

			# x:279 y:119
			OperatableStateMachine.add('Log_No_Connection',
										LogState(text="Have no connection to OCS!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:530 y:133
			OperatableStateMachine.add('Log_Data_Error',
										LogState(text="Received wrong data format!", severity=Logger.REPORT_ERROR),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		# x:335 y:193, x:335 y:112
		_sm_step_back_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['distance'])

		with _sm_step_back_8:
			# x:41 y:63
			OperatableStateMachine.add('Plan_Steps_Back',
										FootstepPlanRelativeState(direction=FootstepPlanRelativeState.DIRECTION_BACKWARD),
										transitions={'planned': 'Execute_Steps_Back', 'failed': 'failed'},
										autonomy={'planned': Autonomy.High, 'failed': Autonomy.High},
										remapping={'distance': 'distance', 'plan_header': 'plan_header'})

			# x:39 y:190
			OperatableStateMachine.add('Execute_Steps_Back',
										ExecuteStepPlanActionState(),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'plan_header': 'plan_header'})


		# x:1156 y:62, x:414 y:199
		_sm_prepare_manipulation_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['template_id', 'grasp_preference', 'hand_side'])

		with _sm_prepare_manipulation_9:
			# x:41 y:61
			OperatableStateMachine.add('Optional_Template_Adjustment',
										LogState(text='Request template adjustment from the operators', severity=Logger.REPORT_HINT),
										transitions={'done': 'Get_Pregrasp'},
										autonomy={'done': Autonomy.High})

			# x:49 y:442
			OperatableStateMachine.add('Plan_To_Pregrasp',
										PlanEndeffectorPoseState(ignore_collisions=False, include_torso=False, allowed_collisions=[], planner_id="RRTConnectkConfigDefault"),
										transitions={'planned': 'Move_To_Pregrasp', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'target_pose': 'pre_grasp', 'hand': 'hand_side', 'joint_trajectory': 'joint_trajectory'})

			# x:365 y:440
			OperatableStateMachine.add('Move_To_Pregrasp',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Request_Hand_Adjustment', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:678 y:54
			OperatableStateMachine.add('Go_to_Grasp',
										_sm_go_to_grasp_0,
										transitions={'finished': 'Request_Stick_Adjustment', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference'})

			# x:49 y:192
			OperatableStateMachine.add('Get_Pregrasp',
										GetTemplatePregraspState(),
										transitions={'done': 'Plan_To_Pregrasp', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'preference': 'grasp_preference', 'pre_grasp': 'pre_grasp'})

			# x:656 y:441
			OperatableStateMachine.add('Request_Hand_Adjustment',
										LogState(text='Request adjustment of the hand', severity=Logger.REPORT_HINT),
										transitions={'done': 'Go_to_Grasp'},
										autonomy={'done': Autonomy.High})

			# x:902 y:57
			OperatableStateMachine.add('Request_Stick_Adjustment',
										LogState(text='Request adjustment of the poking stick', severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.High})


		# x:118 y:674, x:383 y:310, x:455 y:159
		_sm_manipulate_valve_10 = OperatableStateMachine(outcomes=['finished', 'failed', 'pregrasp_again'], input_keys=['template_id', 'hand_side', 'none'])

		with _sm_manipulate_valve_10:
			# x:92 y:25
			OperatableStateMachine.add('Adjust_Wrist',
										_sm_adjust_wrist_3,
										transitions={'finished': 'Insert_Hand'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:739 y:128
			OperatableStateMachine.add('Get_Turning_Affordance',
										GetTemplateAffordanceState(identifier='open'),
										transitions={'done': 'Set_Rotation', 'failed': 'failed', 'not_available': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High, 'not_available': Autonomy.High},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'affordance': 'turning_affordance'})

			# x:85 y:535
			OperatableStateMachine.add('Decide_Repeat',
										OperatorDecisionState(outcomes=['insert_again', 'continue'], hint="Continue or adjust and rotate again", suggestion='continue'),
										transitions={'insert_again': 'Adjust_Wrist', 'continue': 'finished'},
										autonomy={'insert_again': Autonomy.High, 'continue': Autonomy.High})

			# x:744 y:304
			OperatableStateMachine.add('Plan_Turning_Affordance',
										PlanAffordanceState(vel_scaling=0.1, planner_id="RRTConnectkConfigDefault"),
										transitions={'done': 'Execute_Turning_Affordance', 'incomplete': 'Execute_Turning_Affordance', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'incomplete': Autonomy.High, 'failed': Autonomy.High},
										remapping={'affordance': 'turning_affordance', 'hand': 'hand_side', 'reference_point': 'none', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:324 y:25
			OperatableStateMachine.add('Insert_Hand',
										_sm_insert_hand_2,
										transitions={'finished': 'Decide_Ready_or_Retry'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})

			# x:744 y:531
			OperatableStateMachine.add('Extract_Hand',
										_sm_extract_hand_1,
										transitions={'finished': 'Decide_Repeat', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'template_id': 'template_id', 'none': 'none'})

			# x:771 y:215
			OperatableStateMachine.add('Set_Rotation',
										CalculationState(calculation=self.reduce_rotation_angle),
										transitions={'done': 'Plan_Turning_Affordance'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'turning_affordance', 'output_value': 'turning_affordance'})

			# x:736 y:406
			OperatableStateMachine.add('Execute_Turning_Affordance',
										ExecuteTrajectoryMsgState(controller=arm_controller),
										transitions={'done': 'Decide_Keep_Turning', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:994 y:406
			OperatableStateMachine.add('Decide_Keep_Turning',
										OperatorDecisionState(outcomes=['retract', 'turn_more'], hint='If OK, keep turning the valve!', suggestion=None),
										transitions={'retract': 'Extract_Hand', 'turn_more': 'Plan_Turning_Affordance'},
										autonomy={'retract': Autonomy.High, 'turn_more': Autonomy.High})

			# x:599 y:28
			OperatableStateMachine.add('Decide_Ready_or_Retry',
										OperatorDecisionState(outcomes=['pregrasp_again', 'tempalte_ready'], hint='Either adjust the template or (if something went wrong) go back to pregrasp.', suggestion='template_ready'),
										transitions={'pregrasp_again': 'pregrasp_again', 'tempalte_ready': 'Get_Turning_Affordance'},
										autonomy={'pregrasp_again': Autonomy.High, 'tempalte_ready': Autonomy.Low})



		with _state_machine:
			# x:51 y:195
			OperatableStateMachine.add('Notify_Execution_Started',
										LogState(text='Execution has started. Consider making modifications if this is a re-run.', severity=Logger.REPORT_HINT),
										transitions={'done': 'Request_Template'},
										autonomy={'done': Autonomy.Full})

			# x:823 y:330
			OperatableStateMachine.add('Manipulate_Valve',
										_sm_manipulate_valve_10,
										transitions={'finished': 'Retract_Arms', 'failed': 'Manipulate_Valve', 'pregrasp_again': 'Manipulate_Valve'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'pregrasp_again': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'hand_side': 'hand_side', 'none': 'none'})

			# x:737 y:64
			OperatableStateMachine.add('Prepare_Manipulation',
										_sm_prepare_manipulation_9,
										transitions={'finished': 'Turn_Valve', 'failed': 'Turn_Valve_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'template_id': 'template_id', 'grasp_preference': 'default_preference', 'hand_side': 'hand_side'})

			# x:253 y:16
			OperatableStateMachine.add('Decide_Walking',
										OperatorDecisionState(outcomes=['walk', 'skip'], hint="Walk to the valve?", suggestion='walk'),
										transitions={'walk': 'Walk to Template', 'skip': 'Face_Valve'},
										autonomy={'walk': Autonomy.High, 'skip': Autonomy.Full})

			# x:1030 y:529
			OperatableStateMachine.add('Step_Back',
										_sm_step_back_8,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'distance': 'step_back_distance'})

			# x:1019 y:408
			OperatableStateMachine.add('Decide_Step_Back',
										OperatorDecisionState(outcomes=["stand", "step_back"], hint="Should the robot step back from the valve?", suggestion="step_back"),
										transitions={'stand': 'finished', 'step_back': 'Step_Back'},
										autonomy={'stand': Autonomy.Full, 'step_back': Autonomy.High})

			# x:250 y:122
			OperatableStateMachine.add('Walk to Template',
										self.use_behavior(WalktoTemplateSM, 'Walk to Template'),
										transitions={'finished': 'Face_Valve', 'failed': 'failed', 'aborted': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'grasp_preference': 'default_preference', 'hand_side': 'hand_side', 'template_id': 'template_id'})

			# x:53 y:79
			OperatableStateMachine.add('Request_Template',
										_sm_request_template_7,
										transitions={'finished': 'Decide_Walking', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'none': 'none', 'template_id': 'template_id'})

			# x:1023 y:203
			OperatableStateMachine.add('Retract_Arms',
										_sm_retract_arms_6,
										transitions={'finished': 'Decide_Step_Back', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side', 'torso_center': 'torso_center'})

			# x:752 y:209
			OperatableStateMachine.add('Turn_Valve_Manually',
										LogState(text='Have the operator turn the valve manually.', severity=Logger.REPORT_HINT),
										transitions={'done': 'Retract_Arms'},
										autonomy={'done': Autonomy.Full})

			# x:504 y:63
			OperatableStateMachine.add('Face_Valve',
										_sm_face_valve_5,
										transitions={'finished': 'Prepare_Manipulation', 'failed': 'Turn_Valve_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'hand_side': 'hand_side'})

			# x:1026 y:63
			OperatableStateMachine.add('Turn_Valve',
										_sm_turn_valve_4,
										transitions={'finished': 'Retract_Arms', 'failed': 'Turn_Valve_Manually'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'stick_reference': 'stick_reference', 'template_id': 'template_id', 'hand_side': 'hand_side'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	def reduce_rotation_angle(self, affordance):
		degrees = math.copysign(self._turn_amount, affordance.displacement)
		affordance.displacement = math.radians(degrees)
		affordance.keep_orientation = self._keep_orientation
		Logger.loginfo("Turning by %d degrees. Keep orientation: %s" % (degrees, str(self._keep_orientation)))
		return affordance
	
	# [/MANUAL_FUNC]
